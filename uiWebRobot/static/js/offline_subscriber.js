window.addEventListener("load", () => {
    /*if ('serviceWorker' in navigator) {   
        navigator.serviceWorker.register('/sw.js').then(function() {
            alert('Service Worker registration was successful !');
        });
    }else{*/
        var myWorker = new Worker('/sw.js');
    //}
});