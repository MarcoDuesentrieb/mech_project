// Wir definieren eine Variable für die PeerConnection
var pc = null;

// Funktion, um eine Verbindung zu starten und zu verhandeln
async function negotiate() 
{
    try 
    {
        // Füge Video- und Audio-Transceiver hinzu (die nur empfangen sollen)
        pc.addTransceiver('video', { direction: 'recvonly' });
        pc.addTransceiver('audio', { direction: 'recvonly' });

        // Erstelle das Angebot (Offer)
        let offer = await pc.createOffer();
        
        // Setze die lokale Beschreibung des Angebots
        await pc.setLocalDescription(offer);

        // Warte darauf, dass das ICE Gathering abgeschlossen ist
        await new Promise(function(resolve) 
        {
            if (pc.iceGatheringState === 'complete') 
            {
                resolve();
            } 
            else 
            {
                const checkState = function() 
                {
                    if (pc.iceGatheringState === 'complete') 
                    {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });

        // Sende das Angebot an den Server
        let response = await fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });

        // Empfange die Antwort des Servers
        let answer = await response.json();
        
        // Setze die entfernte Beschreibung (Antwort vom Server)
        await pc.setRemoteDescription(answer);

    } catch (e) {
        // Falls ein Fehler auftritt, zeige eine Fehlermeldung
        alert(e);
    }
}

// Funktion, um die Verbindung zu starten
function start() 
{          
    // Konfiguriere den PeerConnection mit einer "unified-plan" SDP-Semantik
    var config = { sdpSemantics: 'unified-plan' };

    // Erstelle eine neue PeerConnection
    pc = new RTCPeerConnection(config);
        
    // Füge einen EventListener hinzu, der die Audio-/Video-Streams anzeigt
    pc.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            // Zeige das Video im <video>-Element an
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            // Zeige den Audio-Stream im <audio>-Element an
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    // Verstecke den "Start"-Button und zeige den "Stop"-Button
    document.getElementById('start').style.display = 'none';
    document.getElementById('stop').style.display = 'inline-block';
    
    // Starte die Verhandlung der Verbindung
    negotiate();
}

// Funktion, um die Verbindung zu stoppen
function stop() {
    // Verstecke den "Stop"-Button und zeige den "Start"-Button
    document.getElementById('stop').style.display = 'none';
    document.getElementById('start').style.display = 'inline-block';

    // Schließe die PeerConnection nach einer kurzen Verzögerung
    setTimeout(function() {
        pc.close();
    }, 500);
}

// Funktion, um den Fullscreen-Modus für das Video zu aktivieren
function toggleFullscreen() {
    const videoElement = document.getElementById('video');

    // Überprüfe, welche Methode für Fullscreen im Browser verwendet wird
    if (videoElement.requestFullscreen) {
        videoElement.requestFullscreen();
    } else if (videoElement.mozRequestFullScreen) { // Firefox
        videoElement.mozRequestFullScreen();
    } else if (videoElement.webkitRequestFullscreen) { // Chrome, Safari und Opera
        videoElement.webkitRequestFullscreen();
    } else if (videoElement.msRequestFullscreen) { // IE/Edge
        videoElement.msRequestFullscreen();
    }
}

