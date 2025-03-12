const express = require('express');
const fs = require('fs');
const path = require('path');
const https = require('https');

const app = express();
const PORT = 3000;

// Load SSL certificate and key
const privateKey = fs.readFileSync('key.pem', 'utf8');
const certificate = fs.readFileSync('cert.pem', 'utf8');
const credentials = { key: privateKey, cert: certificate };

app.use(express.json());

// Serve static files (HTML, CSS, JS)
app.use(express.static(path.join(__dirname, 'public')));

// Custom route for the main HTML file
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'camera.html'));
});

// API endpoint to receive data from the client
app.post('/data', (req, res) => {
    const data = req.body;
    console.log('Received data:', data); // 添加这行来调试
    console.log('KFimuData exists?', 'KFimuData' in data); // 检查字段是否存在
    const timestamp = new Date().toISOString();
    
    const logEntry = `
    Timestamp: ${timestamp}
    imuData:${JSON.stringify(data.imuData)}
    KFimuData:${JSON.stringify(data.KFimuData)}
    IMU Timestamps: ${JSON.stringify(data.timestamps)}
    `;
    // Save the data to a file
    fs.appendFile('output.txt', logEntry + '\n', (err) => {
        if (err) {
            console.error('Failed to save data:', err);
            res.status(500).send('Failed to save data');
        } else {
            //console.log('Data saved successfully');
            res.status(200).send('Data saved successfully');
        }
    });
});

// Create HTTPS server
const httpsServer = https.createServer(credentials, app);

httpsServer.listen(PORT, () => {
    console.log(`Server running on https://localhost:${PORT}`);
});