function cheapRuler(lat, units) {
    return new CheapRuler(lat, units);
}

cheapRuler.fromTile = function (y, z, units) {
    var n = Math.PI * (1 - 2 * (y + 0.5) / Math.pow(2, z));
    var lat = Math.atan(0.5 * (Math.exp(n) - Math.exp(-n))) * 180 / Math.PI;
    return new CheapRuler(lat, units);
};

function CheapRuler(lat, units) {
    if (lat === undefined) throw new Error('No latitude given.');

    // units per degree on equator
    this.d = (units === 'miles' ? 24901.55 : 40075.16) / 360;

    // longitude correction based on latitude
    this.e = Math.cos(lat * Math.PI / 180);
}

CheapRuler.prototype = {
    distance: function (a, b) {
        var dx = (a[0] - b[0]) * this.e;
        var dy = a[1] - b[1];
        return Math.sqrt(dx * dx + dy * dy) * this.d;
    },

    bearing: function (a, b) {
        var dx = (b[0] - a[0]) * this.e;
        var dy = b[1] - a[1];
        if (!dx && !dy) return 0;
        var bearing = Math.atan2(-dy, dx) * 180 / Math.PI + 90;
        if (bearing > 180) bearing -= 360;
        return bearing;
    },

    destination: function (p, dist, bearing) {
        var a = (90 - bearing) * Math.PI / 180;
        var d = dist / this.d;
        return [
            p[0] + d * Math.cos(a) / this.e,
            p[1] + d * Math.sin(a)
        ];
    },

    lineDistance: function (points) {
        var total = 0;
        for (var i = 0; i < points.length - 1; i++) {
            total += this.distance(points[i], points[i + 1]);
        }
        return total;
    },

    area: function (polygon) {
        var sum = 0;

        for (var i = 0; i < polygon.length; i++) {
            var ring = polygon[i];

            for (var j = 0, len = ring.length, k = len - 1; j < len; k = j++) {
                sum += (ring[j][0] - ring[k][0]) * (ring[j][1] + ring[k][1]) * (i ? -1 : 1);
            }
        }

        return (Math.abs(sum) / 2) * this.e * this.d * this.d;
    },

    along: function (line, dist) {
        var sum = 0;

        if (dist <= 0) return line[0];

        for (var i = 0; i < line.length - 1; i++) {
            var p0 = line[i];
            var p1 = line[i + 1];
            var d = this.distance(p0, p1);
            sum += d;
            if (sum > dist) return interpolate(p0, p1, (dist - (sum - d)) / d);
        }

        return line[line.length - 1];
    },

    pointOnLine: function (line, p) {
        var minDist = Infinity;
        var minX, minY, minI, minT;

        for (var i = 0; i < line.length - 1; i++) {

            var x = line[i][0];
            var y = line[i][1];
            var dx = (line[i + 1][0] - x) * this.e;
            var dy = line[i + 1][1] - y;

            if (dx !== 0 || dy !== 0) {

                var t = ((p[0] - x) * this.e * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);

                if (t > 1) {
                    x = line[i + 1][0];
                    y = line[i + 1][1];

                } else if (t > 0) {
                    x += dx * t / this.e;
                    y += dy * t;
                }
            }

            dx = (p[0] - x) * this.e;
            dy = p[1] - y;

            var sqDist = dx * dx + dy * dy;
            if (sqDist < minDist) {
                minDist = sqDist;
                minX = x;
                minY = y;
                minI = i;
                minT = t;
            }
        }

        return {
            point: [minX, minY],
            index: minI,
            t: minT
        };
    },

    lineSlice: function (start, stop, line) {
        var p1 = this.pointOnLine(line, start);
        var p2 = this.pointOnLine(line, stop);

        if (p1.index > p2.index || (p1.index === p2.index && p1.t > p2.t)) {
            var tmp = p1;
            p1 = p2;
            p2 = tmp;
        }

        var slice = [p1.point];

        var l = p1.index + 1;
        var r = p2.index;

        if (!equals(line[l], slice[0]) && l <= r)
            slice.push(line[l]);

        for (var i = l + 1; i <= r; i++) {
            slice.push(line[i]);
        }

        if (!equals(line[r], p2.point))
            slice.push(p2.point);

        return slice;
    },

    lineSliceAlong: function (start, stop, line) {
        var sum = 0;
        var slice = [];

        for (var i = 0; i < line.length - 1; i++) {
            var p0 = line[i];
            var p1 = line[i + 1];
            var d = this.distance(p0, p1);

            sum += d;

            if (sum > start && slice.length === 0) {
                slice.push(interpolate(p0, p1, (start - (sum - d)) / d));
            }

            if (sum >= stop) {
                slice.push(interpolate(p0, p1, (stop - (sum - d)) / d));
                return slice;
            }

            if (sum > start) slice.push(p1);
        }

        return slice;
    },

    bufferPoint: function (p, buffer) {
        var v = buffer / this.d;
        var h = v / this.e;
        return [
            p[0] - h,
            p[1] - v,
            p[0] + h,
            p[1] + v
        ];
    },

    bufferBBox: function (bbox, buffer) {
        var v = buffer / this.d;
        var h = v / this.e;
        return [
            bbox[0] - h,
            bbox[1] - v,
            bbox[2] + h,
            bbox[3] + v
        ];
    },

    insideBBox: function (p, bbox) {
        return p[0] >= bbox[0] &&
            p[0] <= bbox[2] &&
            p[1] >= bbox[1] &&
            p[1] <= bbox[3];
    }
};

function equals(a, b) {
    return a[0] === b[0] && a[1] === b[1];
}

function interpolate(a, b, t) {
    var dx = b[0] - a[0];
    var dy = b[1] - a[1];
    return [
        a[0] + dx * t,
        a[1] + dy * t
    ];
}

const rulerCache = {};

function getRuler(latitude) {
    // Cache rulers every 0.00001 degrees of latitude
    const roundedLatitude = Math.round(latitude * 100000);
    if (rulerCache[roundedLatitude] === undefined) {
        rulerCache[roundedLatitude] = cheapRuler(latitude, 'meters');
    }
    return rulerCache[roundedLatitude];
}

// Distance between two points in metres
function getDist(p1, p2) {
    getRuler(p1[1]).distance(p1, p2);
}

// Distance from a point to a segment (line between two points) in metres
function getSegDist(p, p1, p2) {
    const ruler = getRuler(p[1]);
    const pointOnLine = ruler.pointOnLine([p1, p2], p).point;
    return ruler.distance(p, pointOnLine);
}

function simplifyDPStep(points, first, last, offsetTolerance, gapTolerance, simplified) {
    let maxDistanceFound = offsetTolerance,
        index;

    for (let i = first + 1; i < last; i++) {
        const distance = getSegDist(points[i], points[first], points[last]);

        if (distance > maxDistanceFound) {
            index = i;
            maxDistanceFound = distance;
        }
    }

    // Don't remove a point if it would create a segment longer
    // than gapTolerance
    const firstLastDist = getDist(points[first], points[last]);

    if (maxDistanceFound > offsetTolerance || firstLastDist > gapTolerance) {
        if (index - first > 1) simplifyDPStep(points, first, index, offsetTolerance, gapTolerance, simplified);
        simplified.push(points[index]);
        if (last - index > 1) simplifyDPStep(points, index, last, offsetTolerance, gapTolerance, simplified);
    }
}

// simplification using Ramer-Douglas-Peucker algorithm
function simplifyDouglasPeucker(points, offsetTolerance, gapTolerance) {
    const last = points.length - 1;
    const simplified = [points[0]];
    simplifyDPStep(points, 0, last, offsetTolerance, gapTolerance, simplified);
    simplified.push(points[last]);
    return simplified;
}

function simplify(points, offsetTolerance, gapTolerance) {
    if (points.length <= 2) return points;
    points = simplifyDouglasPeucker(points, offsetTolerance, gapTolerance);
    return points;
}