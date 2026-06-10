// Demo Javascript app for negotiating and streaming a sendrecv webrtc stream
// with a GStreamer app. Runs only in passive mode, i.e., responds to offers
// with answers, exchanges ICE candidates, and streams.

const ws_server = undefined; // Set this to override the automatic detection
const ws_port = undefined;

const ws_server_local = window.location.hostname === "sn015-nano.natuition.vpn"
  ? "sn015-orin.natuition.vpn"
  : "192.168.9.99";
const ws_port_local = "8443";

const rtc_configuration = {
  iceServers: [
    { urls: "stun:stun.l.google.com:19302" },
  ],
};

const sessions = {};

const getOurId = () =>
  "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, c => {
    const r = (Math.random() * 16) | 0;
    return (c === "x" ? r : (r & 0x3) | 0x8).toString(16);
  });

const Uint8ToString = u8a => {
  const CHUNK_SZ = 0x8000;
  const c = [];
  for (let i = 0; i < u8a.length; i += CHUNK_SZ) {
    c.push(String.fromCharCode(...u8a.subarray(i, i + CHUNK_SZ)));
  }
  return c.join("");
};

class Session {
  constructor(our_id, peer_id, closed_callback) {
    this.id = null;
    this.peer_connection = null;
    this.ws_conn = null;
    this.peer_id = peer_id;
    this.our_id = our_id;
    this.closed_callback = closed_callback;
    this.data_channel = null;
    this.input = null;

    document
      .getElementById("stream")
      .addEventListener("playing", this.streamIsPlaying.bind(this), false);

    this.websocketServerConnect();
  }

  getVideoElement = () => document.getElementById("stream");

  resetState = () => {
    if (this.peer_connection) {
      this.peer_connection.close();
      this.peer_connection = null;
    }
    const videoElement = this.getVideoElement();
    if (videoElement) {
      videoElement.pause();
      videoElement.src = "";
      videoElement.srcObject = null;
    }

    if (this.ws_conn) {
      this.ws_conn.close();
      this.ws_conn = null;
    }

    this.input?.detach();
    this.data_channel = null;

  };

  handleIncomingError = error => {
    this.resetState();
    this.closed_callback(this.our_id);
  };

  setStatus = text => console.log(text);

  setError = text => {
    console.error(text);
    const span = document.getElementById(`status-${this.our_id}`);
    if (span) {
      span.textContent = text;
      span.classList.add("error");
    }
    this.resetState();
    this.closed_callback(this.our_id);
  };

  onLocalDescription = desc => {
    console.log("Got local description: ", desc, this);
    this.peer_connection
      .setLocalDescription(desc)
      .then(() => {
        this.setStatus("Sending SDP answer");
        this.ws_conn.send(
          JSON.stringify({
            type: "peer",
            sessionId: this.id,
            sdp: this.peer_connection.localDescription.toJSON(),
          })
        );
      })
      .catch(this.setError);
  };

  onRemoteDescriptionSet = () => {
    this.setStatus("Remote SDP set");
    this.setStatus("Got SDP offer");
    this.peer_connection
      .createAnswer()
      .then(this.onLocalDescription)
      .catch(this.setError);
  };

  onIncomingSDP = sdp => {
    this.peer_connection
      .setRemoteDescription(sdp)
      .then(this.onRemoteDescriptionSet)
      .catch(this.setError);
  };

  onIncomingICE = ice => {
    this.peer_connection
      .addIceCandidate(new RTCIceCandidate(ice))
      .catch(this.setError);
  };

  onServerMessage = event => {
    //console.log("Received ", event.data);
    let msg;
    try {
      msg = JSON.parse(event.data);
    } catch (e) {
      this.handleIncomingError(`Error parsing incoming JSON: ${event.data}`);
      return;
    }

    switch (msg.type) {
      case "registered":
        this.setStatus("Registered with server");
        this.connectPeer();
        break;
      case "sessionStarted":
        this.setStatus("Registered with server");
        this.id = msg.sessionId;
        break;
      case "error":
        this.handleIncomingError(msg.details);
        break;
      case "endSession":
        this.resetState();
        this.closed_callback(this.our_id);
        break;
      case "peer":
        if (!this.peer_connection) this.createCall(msg);
        if (msg.sdp) this.onIncomingSDP(msg.sdp);
        else if (msg.ice) this.onIncomingICE(msg.ice);
        else this.handleIncomingError(`Unknown incoming JSON: ${msg}`);
        break;
      case "welcome":
        console.info("Got welcomed with ID", msg.peer_id);
        break;
      default:
        console.error("Unsupported message: ", msg);
    }
  };

  streamIsPlaying = () => {
    this.setStatus("Streaming");
    document.getElementById("no_cam").style.display = "none";
    this.getVideoElement().setAttribute("controls", "");
  }

  onServerClose = () => {
    this.resetState();
    this.closed_callback(this.our_id);
  };

  onServerError = () => this.handleIncomingError("Server error");

  websocketServerConnect = () => {
    console.log("Our ID:", this.our_id);

    const ws_url = `ws://${ws_server_local}:${ws_port_local}`;
    this.setStatus(`Connecting to server ${ws_url}`);

    this.ws_conn = new WebSocket(ws_url);
    this.ws_conn.addEventListener("open", () => {
      this.setStatus("Connecting to the peer");
      this.connectPeer();
    });
    this.ws_conn.addEventListener("error", this.onServerError);
    this.ws_conn.addEventListener("message", this.onServerMessage);
    this.ws_conn.addEventListener("close", this.onServerClose);
  };

  connectPeer = () => {
    this.setStatus(`Connecting ${this.peer_id}`);
    this.ws_conn.send(JSON.stringify({ type: "startSession", peerId: this.peer_id }));
  };

  onRemoteStreamAdded = event => {
    const videoTracks = event.stream.getVideoTracks();
    const audioTracks = event.stream.getAudioTracks();

    if (videoTracks.length > 0) {
      this.getVideoElement().srcObject = event.stream;
      this.getVideoElement().play();
    } else {
      this.handleIncomingError("Stream with unknown tracks added, resetting");
    }
  };

  createCall = msg => {
    console.log("Creating RTCPeerConnection");
    this.peer_connection = new RTCPeerConnection(rtc_configuration);
    this.peer_connection.onaddstream = this.onRemoteStreamAdded;

    this.peer_connection.ondatachannel = event => {
      console.log(`Data channel created: ${event.channel.label}`);
      this.data_channel = event.channel;

      const video_element = this.getVideoElement();
      if (video_element) {
        this.input = new Input(video_element, data => {
          this.data_channel?.send(JSON.stringify(data));
        });
      }

      this.data_channel.onopen = () => this.input?.attach();
      this.data_channel.onclose = () => {
        this.input?.detach();
        this.data_channel = null;
      };
      this.data_channel.onerror = event => {
        this.input?.detach();
        console.warn("Error on receive channel", event.data);
        this.data_channel = null;
      };

      let buffer = [];
      this.data_channel.onmessage = event => {
        if (typeof event.data === "string") {
          if (event.data === "BEGIN_IMAGE") buffer = [];
          else if (event.data === "END_IMAGE") {
            const decoder = new TextDecoder("ascii");
            const str = decoder.decode(new Uint8Array(buffer));
            document.getElementById("image").src = `data:image/png;base64, ${str}`;
          }
        } else {
          const view = new DataView(event.data);
          for (let i = 0; i < view.byteLength; i++) buffer.push(view.getUint8(i));
        }
      };
    };

    this.peer_connection.onicecandidate = event => {
      if (!event.candidate) return;
      this.ws_conn.send(
        JSON.stringify({ type: "peer", sessionId: this.id, ice: event.candidate.toJSON() })
      );
    };
    this.setStatus("Created peer connection for call, waiting for SDP");
  };
}

// Global functions
const startSession = () => {
  const peer_id = document.getElementById("camera-id").value;
  if (!peer_id) return;
  sessions[peer_id] = new Session(peer_id);
};

const session_closed = peer_id => {
  sessions[peer_id] = null;
  document.getElementById("no_cam").style.display = "block";
  document.getElementById("stream").removeAttribute("controls");
};

const addPeer = (peer_id, meta = { "display-name": peer_id }) => {
  sessions[peer_id] = new Session(getOurId(), peer_id, session_closed);
};

const clearPeers = () => console.log("Clearing peers");

const onServerMessage = event => {
  let msg;
  try {
    msg = JSON.parse(event.data);
  } catch {
    console.error("Error parsing incoming JSON: " + event.data);
    return;
  }

  switch (msg.type) {
    case "welcome":
      console.info(`Got welcomed with ID ${msg.peer_id}`);
      ws_conn.send(JSON.stringify({ type: "list" }));
      break;
    case "list":
      clearPeers();
      msg.producers.forEach(p => addPeer(p.id, p.meta));
      break;
    case "peerStatusChanged":
      const li = document.getElementById(`peer-${msg.peerId}`);
      if (msg.roles.includes("producer") && !li) addPeer(msg.peerId, msg.meta);
      else if (li) li.remove();
      break;
    default:
      console.error("Unsupported message: ", msg);
  }
};

const clearConnection = () => {
  ws_conn.removeEventListener("error", onServerError);
  ws_conn.removeEventListener("message", onServerMessage);
  ws_conn.removeEventListener("close", onServerClose);
  ws_conn = null;
};

const onServerClose = () => {
  clearConnection();
  clearPeers();
  setTimeout(connect, 1000);
};

const onServerError = () => {
  clearConnection();
  clearPeers();
  setTimeout(connect, 1000);
};

const connect = () => {
  const ws_url = `ws://${ws_server_local}:${ws_port_local}`;
  ws_conn = new WebSocket(ws_url);
  ws_conn.addEventListener("open", () => {
    ws_conn.send(JSON.stringify({ type: "setPeerStatus", roles: ["listener"] }));
  });
  ws_conn.addEventListener("error", onServerError);
  ws_conn.addEventListener("message", onServerMessage);
  ws_conn.addEventListener("close", onServerClose);
};

const web_rtc_connect = () => connect();
