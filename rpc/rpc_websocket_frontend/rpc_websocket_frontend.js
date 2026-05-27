

console.log("hello");

// Create WebSocket connection.
const socket = new WebSocket('ws://localhost:8001/');

const connectionPromise = new Promise((resolve, reject) => {
socket.onopen = () => {
    console.log("connected");
    resolve(socket);


};
socket.onerror = error => {
    console.error(error);

}

socket.onclose = () => {
    console.error("Websocket connection closed!"); 
}
});

// Connection opened
socket.addEventListener('open', function (event) {
    socket.send('Hello Server!');
});

let logBox = document.getElementById('serial-messages');
// Listen for messages
socket.addEventListener('message', function (event) {
    console.log('Message from serial:', event.data);
    // Add message to log
    let logLine = document.createElement('p');
    logLine.innerText = event.data;
    logBox.append(logLine);
});


function callRPC(name) {
    console.log("calling: " + name);
    socket.send(`{"name": "${name}"}`);
}

const speed = 15.0;
function callGoTo() {
    let x = document.getElementById('x').value;
    let y = document.getElementById('y').value;
    console.log("calling: go to at: " + x + ", " + y);
    socket.send(`{"name": "go_to_xy", "args": [${x}, ${y}, ${speed}]}`);
}