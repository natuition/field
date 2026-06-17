// Demo Javascript app for negotiating and streaming a sendrecv webrtc stream
// with a GStreamer app. Runs only in passive mode, i.e., responds to offers
// with answers, exchanges ICE candidates, and streams.

const ws_port_local = "8443";
const ws_server_local = window.location.hostname === "sn015-nano.natuition.vpn"
  ? "sn015-orin.natuition.vpn" : "192.168.9.99";

const rtc_configuration = {
  iceServers: [
    { urls: "stun:stun.l.google.com:19302" },
  ],
};

const sessions = {};
let ws_conn = null;

const getWebSocketScheme = () =>
  window.location.protocol === "https:" ? "wss" : "ws";

const logStep = (scope, message, details) => {
  const timestamp = new Date().toISOString();
  if (details === undefined) {
    console.log(`[${timestamp}] [${scope}] ${message}`);
    return;
  }
  console.log(`[${timestamp}] [${scope}] ${message}`, details);
};

const logWarn = (scope, message, details) => {
  const timestamp = new Date().toISOString();
  if (details === undefined) {
    console.warn(`[${timestamp}] [${scope}] ${message}`);
    return;
  }
  console.warn(`[${timestamp}] [${scope}] ${message}`, details);
};

const logError = (scope, message, details) => {
  const timestamp = new Date().toISOString();
  if (details === undefined) {
    console.error(`[${timestamp}] [${scope}] ${message}`);
    return;
  }
  console.error(`[${timestamp}] [${scope}] ${message}`, details);
};

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
    this.remote_stream_attached = false;
    this.onVideoPlaying = this.streamIsPlaying.bind(this);

    logStep(
      `Session:${this.peer_id}`,
      "Construction de session: creation de l'objet Session, attente de negotiation WebRTC",
      { our_id: this.our_id, peer_id: this.peer_id }
    );

    document
      .getElementById("stream")
      .addEventListener("playing", this.onVideoPlaying, false);

    const videoElement = this.getVideoElement();
    if (videoElement) {
      // Mobile browsers (especially iOS Safari) require these flags for inline autoplay.
      videoElement.autoplay = true;
      videoElement.playsInline = true;
      videoElement.muted = true;
      videoElement.setAttribute("autoplay", "true");
      videoElement.setAttribute("playsinline", "true");
      videoElement.setAttribute("muted", "true");
    }

    this.websocketServerConnect();
  }

  getVideoElement = () => document.getElementById("stream");

  attachRemoteStreamOnce = (stream, source) => {
    if (!stream) return;
    if (this.remote_stream_attached) {
      logStep(
        `Session:${this.peer_id}`,
        "Flux distant deja attache, evenement ignore",
        { source }
      );
      return;
    }

    const videoTracks = stream.getVideoTracks();
    if (videoTracks.length === 0) {
      logWarn(
        `Session:${this.peer_id}`,
        "Evenement media sans piste video, attente d'un stream valide",
        { source }
      );
      return;
    }

    this.remote_stream_attached = true;
    logStep(
      `Session:${this.peer_id}`,
      "Attachement du flux distant sur l'element video",
      { source, videoTracks: videoTracks.length }
    );
    this.attachStreamToVideo(stream);
  };

  attachStreamToVideo = stream => {
    const videoElement = this.getVideoElement();
    if (!videoElement) {
      logWarn(`Session:${this.peer_id}`, "Element video introuvable pour attacher le stream");
      return;
    }
    videoElement.srcObject = stream;
    logStep(
      `Session:${this.peer_id}`,
      "Flux video attache a l'element HTMLVideoElement puis tentative de play()"
    );

    const playPromise = videoElement.play();
    if (playPromise && typeof playPromise.catch === "function") {
      playPromise.catch(err => {
        logWarn(
          `Session:${this.peer_id}`,
          "play() bloque (souvent politique autoplay mobile). Retry sur prochaine interaction utilisateur.",
          err
        );

        const retryPlay = () => {
          videoElement
            .play()
            .then(() => {
              logStep(
                `Session:${this.peer_id}`,
                "play() reussi apres interaction utilisateur"
              );
            })
            .catch(playErr => {
              logError(`Session:${this.peer_id}`, "Echec play() apres interaction utilisateur", playErr);
            })
            .finally(() => {
              window.removeEventListener("touchstart", retryPlay);
              window.removeEventListener("click", retryPlay);
            });
        };

        window.addEventListener("touchstart", retryPlay, { once: true });
        window.addEventListener("click", retryPlay, { once: true });
      });
    }
  };

  resetState = () => {
    logStep(
      `Session:${this.peer_id}`,
      "Reset session: fermeture PeerConnection, WebSocket, DataChannel et nettoyage video"
    );

    if (this.peer_connection) {
      logStep(`Session:${this.peer_id}`, "Fermeture RTCPeerConnection");
      this.peer_connection.close();
      this.peer_connection = null;
    }
    this.remote_stream_attached = false;
    const videoElement = this.getVideoElement();
    if (videoElement) {
      logStep(`Session:${this.peer_id}`, "Arret et nettoyage de la source video HTML");
      videoElement.pause();
      videoElement.src = "";
      videoElement.srcObject = null;
      videoElement.removeEventListener("playing", this.onVideoPlaying, false);
    }

    const session_div = document.getElementById(`session-${this.our_id}`);
    if (session_div) session_div.remove();

    if (this.ws_conn) {
      logStep(`Session:${this.peer_id}`, "Fermeture WebSocket de signalisation");
      this.ws_conn.close();
      this.ws_conn = null;
    }

    this.input?.detach();
    logStep(`Session:${this.peer_id}`, "Detachement des handlers d'input utilisateur");
    this.data_channel = null;
  };

  handleIncomingError = error => {
    logError(`Session:${this.peer_id}`, "Erreur entrante recu, fermeture session", error);
    this.resetState();
    this.closed_callback(this.peer_id);
  };

  setStatus = text => logStep(`Session:${this.peer_id}`, `Status: ${text}`);

  setError = text => {
    logError(`Session:${this.peer_id}`, "Erreur session", text);
    const span = document.getElementById(`status-${this.our_id}`);
    if (span) {
      span.textContent = text;
      span.classList.add("error");
    }
    this.resetState();
    this.closed_callback(this.peer_id);
  };

  onLocalDescription = desc => {
    logStep(
      `Session:${this.peer_id}`,
      "Description locale creee (SDP answer), envoi au serveur de signalisation",
      { type: desc?.type }
    );
    this.peer_connection
      .setLocalDescription(desc)
      .then(() => {
        this.setStatus("Sending SDP answer");
        logStep(
          `Session:${this.peer_id}`,
          "LocalDescription appliquee, emission message peer avec SDP"
        );
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
    logStep(
      `Session:${this.peer_id}`,
      "RemoteDescription appliquee, creation de la SDP answer"
    );
    this.setStatus("Remote SDP set");
    this.setStatus("Got SDP offer");
    this.peer_connection
      .createAnswer()
      .then(this.onLocalDescription)
      .catch(this.setError);
  };

  onIncomingSDP = sdp => {
    logStep(
      `Session:${this.peer_id}`,
      "Reception SDP distante depuis signalisation, application a la PeerConnection",
      { type: sdp?.type }
    );
    this.peer_connection
      .setRemoteDescription(sdp)
      .then(this.onRemoteDescriptionSet)
      .catch(this.setError);
  };

  onIncomingICE = ice => {
    logStep(
      `Session:${this.peer_id}`,
      "Reception ICE candidate distant, ajout a la PeerConnection",
      ice
    );
    this.peer_connection
      .addIceCandidate(new RTCIceCandidate(ice))
      .catch(this.setError);
  };

  onServerMessage = event => {
    logStep(
      `Session:${this.peer_id}`,
      "Message recu depuis WebSocket de signalisation",
      event.data
    );
    let msg;
    try {
      msg = JSON.parse(event.data);
    } catch (e) {
      this.handleIncomingError(`Error parsing incoming JSON: ${event.data}`);
      return;
    }

    switch (msg.type) {
      case "registered":
        logStep(
          `Session:${this.peer_id}`,
          "Serveur confirme l'enregistrement du client. Demarrage startSession"
        );
        this.setStatus("Registered with server");
        this.connectPeer();
        break;
      case "sessionStarted":
        logStep(
          `Session:${this.peer_id}`,
          "Session de signalisation ouverte avec producer",
          { sessionId: msg.sessionId }
        );
        this.setStatus("Registered with server");
        this.id = msg.sessionId;
        break;
      case "error":
        logError(`Session:${this.peer_id}`, "Erreur du serveur de signalisation", msg.details);
        this.handleIncomingError(msg.details);
        break;
      case "endSession":
        logStep(`Session:${this.peer_id}`, "Signal endSession recu, fermeture propre");
        this.resetState();
        this.closed_callback(this.peer_id);
        break;
      case "peer":
        logStep(
          `Session:${this.peer_id}`,
          "Message pair recu: SDP/ICE de negotiation WebRTC",
          { hasSdp: !!msg.sdp, hasIce: !!msg.ice }
        );
        if (!this.peer_connection) this.createCall(msg);
        if (msg.sdp) this.onIncomingSDP(msg.sdp);
        else if (msg.ice) this.onIncomingICE(msg.ice);
        else this.handleIncomingError(`Unknown incoming JSON: ${msg}`);
        break;
      default:
        logWarn(`Session:${this.peer_id}`, "Message de signalisation non supporte", msg);
    }
  };

  streamIsPlaying = () => {
    this.setStatus("Streaming");
    logStep(
      `Session:${this.peer_id}`,
      "Lecture media demarree: les pistes distantes sont attachees a la balise video"
    );
    document.getElementById("no_cam").style.display = "none";
    this.getVideoElement().setAttribute("controls", "");
  };

  onServerClose = () => {
    logWarn(
      `Session:${this.peer_id}`,
      "WebSocket de signalisation ferme. Session locale nettoyee"
    );
    this.resetState();
    this.closed_callback(this.peer_id);
  };

  onServerError = () => {
    logError(`Session:${this.peer_id}`, "Erreur WebSocket de signalisation");
    this.handleIncomingError("Server error");
  };

  websocketServerConnect = () => {
    logStep(
      `Session:${this.peer_id}`,
      "Initialisation connexion WebSocket vers serveur de signalisation",
      { ourId: this.our_id }
    );

    const ws_url = `${getWebSocketScheme()}://${ws_server_local}:${ws_port_local}`;
    this.setStatus(`Connecting to server ${ws_url}`);
    logStep(
      `Session:${this.peer_id}`,
      "Ouverture WebSocket de signalisation",
      { ws_url }
    );

    this.ws_conn = new WebSocket(ws_url);
    this.ws_conn.addEventListener("open", () => {
      this.setStatus("Connecting to the peer");
      logStep(
        `Session:${this.peer_id}`,
        "WebSocket ouverte. Envoi startSession vers le peer cible"
      );
      this.connectPeer();
    });
    this.ws_conn.addEventListener("error", this.onServerError);
    this.ws_conn.addEventListener("message", this.onServerMessage);
    this.ws_conn.addEventListener("close", this.onServerClose);
  };

  connectPeer = () => {
    this.setStatus(`Connecting ${this.peer_id}`);
    logStep(
      `Session:${this.peer_id}`,
      "Emission startSession pour demarrer l'echange SDP/ICE",
      { peerId: this.peer_id }
    );
    this.ws_conn.send(JSON.stringify({ type: "startSession", peerId: this.peer_id }));
  };

  onRemoteStreamAdded = event => {
    logStep(
      `Session:${this.peer_id}`,
      "Flux distant recu via onaddstream, inspection des pistes",
      {
        videoTracks: event.stream.getVideoTracks().length,
        audioTracks: event.stream.getAudioTracks().length,
      }
    );
    this.attachRemoteStreamOnce(event.stream, "onaddstream");
  };

  createCall = msg => {
    logStep(
      `Session:${this.peer_id}`,
      "Creation RTCPeerConnection: debut vrai cycle WebRTC (SDP/ICE/media)",
      { rtc_configuration, msgType: msg?.type }
    );
    this.peer_connection = new RTCPeerConnection(rtc_configuration);
    this.remote_stream_attached = false;
    this.peer_connection.onaddstream = this.onRemoteStreamAdded;
    this.peer_connection.ontrack = event => {
      logStep(
        `Session:${this.peer_id}`,
        "Evenement ontrack recu (compat mobile), attente piste active",
        {
          kind: event.track?.kind,
          streams: event.streams?.length ?? 0,
          muted: event.track?.muted,
          readyState: event.track?.readyState,
        }
      );

      const stream = event.streams && event.streams.length > 0
        ? event.streams[0]
        : null;

      if (!stream) {
        logWarn(
          `Session:${this.peer_id}`,
          "ontrack sans stream associe, attachement impossible"
        );
        return;
      }

      if (event.track && event.track.readyState === "live" && !event.track.muted) {
        this.attachRemoteStreamOnce(stream, "ontrack-live");
      }

      event.track.onunmute = () => {
        this.attachRemoteStreamOnce(stream, "ontrack-onunmute");
      };
    };

    this.peer_connection.ondatachannel = event => {
      logStep(
        `Session:${this.peer_id}`,
        "DataChannel recu depuis le pair",
        { label: event.channel.label }
      );
      this.data_channel = event.channel;

      const video_element = this.getVideoElement();
      if (video_element) {
        logStep(
          `Session:${this.peer_id}`,
          "Initialisation Input pour forward des interactions utilisateur via DataChannel"
        );
        this.input = new Input(video_element, data => {
          logStep(
            `Session:${this.peer_id}`,
            "Envoi evenement utilisateur vers le pair sur DataChannel",
            data
          );
          this.data_channel?.send(JSON.stringify(data));
        });
      }

      this.data_channel.onopen = () => {
        logStep(
          `Session:${this.peer_id}`,
          "DataChannel ouvert: activation capture des inputs utilisateur"
        );
        this.input?.attach();
      };
      this.data_channel.onclose = () => {
        logWarn(
          `Session:${this.peer_id}`,
          "DataChannel ferme: desactivation capture des inputs"
        );
        this.input?.detach();
        this.data_channel = null;
      };
      this.data_channel.onerror = event => {
        logError(`Session:${this.peer_id}`, "Erreur DataChannel", event?.data);
        this.input?.detach();
        this.data_channel = null;
      };

      let buffer = [];
      this.data_channel.onmessage = event => {
        logStep(
          `Session:${this.peer_id}`,
          "Message DataChannel recu",
          { type: typeof event.data }
        );
        if (typeof event.data === "string") {
          if (event.data === "BEGIN_IMAGE") {
            logStep(
              `Session:${this.peer_id}`,
              "Debut d'image segmentee recu sur DataChannel, reset du buffer binaire"
            );
            buffer = [];
          }
          else if (event.data === "END_IMAGE") {
            logStep(
              `Session:${this.peer_id}`,
              "Fin d'image recu: decode Base64 et update de l'element image"
            );
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
      logStep(
        `Session:${this.peer_id}`,
        "Nouveau ICE local genere: envoi au pair via signalisation",
        event.candidate
      );
      this.ws_conn.send(
        JSON.stringify({ type: "peer", sessionId: this.id, ice: event.candidate.toJSON() })
      );
    };
    this.peer_connection.oniceconnectionstatechange = () => {
      logStep(
        `Session:${this.peer_id}`,
        "Changement etat ICE connection",
        this.peer_connection.iceConnectionState
      );
    };
    this.peer_connection.onconnectionstatechange = () => {
      logStep(
        `Session:${this.peer_id}`,
        "Changement etat global PeerConnection",
        this.peer_connection.connectionState
      );
    };
    this.peer_connection.onsignalingstatechange = () => {
      logStep(
        `Session:${this.peer_id}`,
        "Changement etat de signalisation SDP",
        this.peer_connection.signalingState
      );
    };
    this.setStatus("Created peer connection for call, waiting for SDP");
  };
}

// Global functions
const startSession = () => {
  const peer_id = document.getElementById("camera-id").value;
  if (!peer_id) return;
  logStep("Global", "startSession manuel declenche", { peer_id });
  sessions[peer_id] = new Session(peer_id);
};

const session_closed = peer_id => {
  logStep("Global", "Session fermee et retiree du registre local", { peer_id });
  sessions[peer_id] = null;
  document.getElementById("no_cam").style.display = "block";
  document.getElementById("stream").removeAttribute("controls");
};

const addPeer = (peer_id, meta = { "display-name": peer_id }) => {
  logStep("Global", "Ajout d'un peer producer detecte", { peer_id, meta });
  sessions[peer_id] = new Session(getOurId(), peer_id, session_closed);
};

const clearPeers = () => {
  logStep("Global", "Nettoyage liste peers locale");
  Object.keys(sessions).forEach(peer_id => {
    const session = sessions[peer_id];
    if (session && typeof session.resetState === "function") {
      session.resetState();
    }
    sessions[peer_id] = null;
  });
};

const onServerMessage = event => {
  logStep("Global", "Message recu sur WebSocket global", event.data);
  let msg;
  try {
    msg = JSON.parse(event.data);
  } catch {
    logError("Global", "Parsing JSON impossible sur WebSocket global", event.data);
    return;
  }

  switch (msg.type) {
    case "welcome":
      logStep("Global", "Welcome recu: demande de liste des producers", {
        peer_id: msg.peer_id,
      });
      ws_conn.send(JSON.stringify({ type: "list" }));
      break;
    case "list":
      logStep("Global", "Liste des producers recue", {
        count: msg.producers?.length ?? 0,
      });
      clearPeers();
      msg.producers.forEach(p => addPeer(p.id, p.meta));
      break;
    case "peerStatusChanged":
      logStep("Global", "Notification de changement de statut peer", msg);
      const li = document.getElementById(`peer-${msg.peerId}`);
      if (msg.roles.includes("producer") && !li) addPeer(msg.peerId, msg.meta);
      else if (li) li.remove();
      break;
    default:
      logWarn("Global", "Message global non supporte", msg);
  }
};

const clearConnection = () => {
  logStep("Global", "Nettoyage handlers WebSocket global");
  ws_conn.removeEventListener("error", onServerError);
  ws_conn.removeEventListener("message", onServerMessage);
  ws_conn.removeEventListener("close", onServerClose);
  ws_conn = null;
};

const onServerClose = () => {
  logWarn("Global", "WebSocket global ferme. Reconnexion dans 1s");
  clearConnection();
  clearPeers();
  setTimeout(connect, 1000);
};

const onServerError = () => {
  logError("Global", "Erreur WebSocket global. Reconnexion dans 1s");
  clearConnection();
  clearPeers();
  setTimeout(connect, 1000);
};

const connect = () => {
  const ws_url = `${getWebSocketScheme()}://${ws_server_local}:${ws_port_local}`;
  logStep("Global", "Connexion WebSocket globale au serveur", { ws_url });
  ws_conn = new WebSocket(ws_url);
  ws_conn.addEventListener("open", () => {
    logStep("Global", "WebSocket globale ouverte: setPeerStatus listener");
    ws_conn.send(JSON.stringify({ type: "setPeerStatus", roles: ["listener"] }));
  });
  ws_conn.addEventListener("error", onServerError);
  ws_conn.addEventListener("message", onServerMessage);
  ws_conn.addEventListener("close", onServerClose);
};

const setup = () => {
  logStep("Global", "Setup front WebRTC: demarrage sequence de connexion globale");
  connect();
};

const web_rtc_connect = () => setup();