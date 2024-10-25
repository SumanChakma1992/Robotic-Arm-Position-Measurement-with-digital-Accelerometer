const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const bodyParser = require('body-parser');
const { SerialPort, ReadlineParser } = require('serialport');
const path = require('path');

const app = express();
app.use(bodyParser.json());

// Serve the HTML file
app.use(express.static(path.join(__dirname, 'public')));

const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

let wsClient = null;

wss.on('connection', (ws) => {
    console.log('WebSocket client connected');
    wsClient = ws;

    ws.on('message', (message) => {
        console.log('Received message from client:', message);
        if (message === 'toggleMpu1') {
            port.write('toggleMpu1\n');
        } else if (message === 'toggleMpu2') {
            port.write('toggleMpu2\n');
        } else if (message === 'toggleAdxl') {
            port.write('toggleAdxl\n');
        }
    });
});

const serialPortPath = 'COM3'; // Update with your serial port path
const port = new SerialPort({ path: serialPortPath, baudRate: 9600 });
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

parser.on('data', (data) => {
    console.log('Received sensor data:', data);

    // Extract JSON data between <JSON> and </JSON> markers
    const jsonData = data.match(/<JSON>(.*?)<\/JSON>/);
    if (jsonData && jsonData[1]) {
        try {
            const sensorData = JSON.parse(jsonData[1].trim());
            if (sensorData.status) {
                console.log('Initialization status:', sensorData.status);
            } else {
                // Calculate displacement for MPU1 and MPU2
                sensorData.mpu1.displacement = calculateDisplacement(sensorData.mpu1.ax, sensorData.mpu1.ay, sensorData.mpu1.az, 'mpu1');
                sensorData.mpu2.displacement = calculateDisplacement(sensorData.mpu2.ax, sensorData.mpu2.ay, sensorData.mpu2.az, 'mpu2');

                if (wsClient && wsClient.readyState === WebSocket.OPEN) {
                    wsClient.send(JSON.stringify(sensorData));
                }
            }
        } catch (error) {
            console.error('Error parsing JSON:', error);
        }
    } else {
        console.error('Non-JSON data received, ignoring:', data);
    }
});

server.listen(3000, () => {
    console.log('Server is running on port 3000');
});

// Function to calculate displacement based on acceleration
let prevAx = { mpu1: 0, mpu2: 0 }, prevAy = { mpu1: 0, mpu2: 0 }, prevAz = { mpu1: 0, mpu2: 0 };
let velX = { mpu1: 0, mpu2: 0 }, velY = { mpu1: 0, mpu2: 0 }, velZ = { mpu1: 0, mpu2: 0 };
let dispX = { mpu1: 0, mpu2: 0 }, dispY = { mpu1: 0, mpu2: 0 }, dispZ = { mpu1: 0, mpu2: 0 };

// Filtered displacement variables
let filteredDispX = { mpu1: 0, mpu2: 0 };
let filteredDispY = { mpu1: 0, mpu2: 0 };
let filteredDispZ = { mpu1: 0, mpu2: 0 };
const dt = 0.01; // Time step in seconds
const alphaDisp = 0.1; // Low-pass filter constant

function calculateDisplacement(ax, ay, az, mpu) {
    // Calculate velocity from acceleration
    velX[mpu] += 0.5 * (prevAx[mpu] + ax) * dt;
    velY[mpu] += 0.5 * (prevAy[mpu] + ay) * dt;
    velZ[mpu] += 0.5 * (prevAz[mpu] + az) * dt;

    // Calculate displacement from velocity
    dispX[mpu] += velX[mpu] * dt;
    dispY[mpu] += velY[mpu] * dt;
    dispZ[mpu] += velZ[mpu] * dt;

    // Apply low-pass filter to displacement
    filteredDispX[mpu] = alphaDisp * dispX[mpu] + (1 - alphaDisp) * filteredDispX[mpu];
    filteredDispY[mpu] = alphaDisp * dispY[mpu] + (1 - alphaDisp) * filteredDispY[mpu];
    filteredDispZ[mpu] = alphaDisp * dispZ[mpu] + (1 - alphaDisp) * filteredDispZ[mpu];

    // Update previous acceleration values
    prevAx[mpu] = ax;
    prevAy[mpu] = ay;
    prevAz[mpu] = az;

    return { x: filteredDispX[mpu], y: filteredDispY[mpu], z: filteredDispZ[mpu] };
}
