const express = require("express");
const fs = require("fs");
const path = require("path");
const https = require("https");

const app = express();
const PORT = 3000;

// Load SSL certificate and key
const privateKey = fs.readFileSync("key.pem", "utf8");
const certificate = fs.readFileSync("cert.pem", "utf8");
const credentials = { key: privateKey, cert: certificate };

app.use(express.json());

// Serve static files (HTML, CSS, JS)
app.use(express.static(path.join(__dirname, "public")));

// Custom route for the main HTML file
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "public", "camera.html"));
});

// 定义 CSV 文件路径
const csvFilePath = path.join(__dirname, "output.csv");

// 检查文件是否存在，如果不存在则写入列名
if (!fs.existsSync(csvFilePath)) {
  const header =
    "timestamp,\
imu_quaternion_x,imu_quaternion_y,imu_quaternion_z,imu_quaternion_w,\
imu_angular_velocity_x,imu_angular_velocity_y,imu_angular_velocity_z,\
imu_acceleration_x,imu_acceleration_y,imu_acceleration_z,\
kf_quaternion_x,kf_quaternion_y,kf_quaternion_z,kf_quaternion_w,\
kf_angular_velocity_x,kf_angular_velocity_y,kf_angular_velocity_z,\
kf_acceleration_x,kf_acceleration_y,kf_acceleration_z\n";
  fs.writeFileSync(csvFilePath, header); // 首次写入列名
}

// API endpoint to receive data from the client
app.post("/data", (req, res) => {
  const data = req.body;
  console.log("Received data:", data);
  // 定义转换函数：将数组转换为逗号分隔的字符串（去除方括号）
  const arrayToCsvString = (arr) => {
    if (!Array.isArray(arr)) return ""; // 处理可能的 null/undefined
    return arr.map((num) => (num === null ? "" : num.toString())).join(","); // 处理 null 值
  };
  const csvLine =
    [
      JSON.stringify(data.timestamp),
      arrayToCsvString(data.imuData.quaternion),
      arrayToCsvString(data.imuData.angularVelocity),
      arrayToCsvString(data.imuData.acceleration),
      arrayToCsvString(data.KFimuData.quaternion),
      arrayToCsvString(data.KFimuData.angularVelocity),
      arrayToCsvString(data.KFimuData.acceleration),
    ].join(",") + "\n";

  fs.appendFile("output.csv", csvLine, (err) => {
    if (err) {
      console.error("Failed to save data:", err);
      res.status(500).send("Failed to save data");
    } else {
      res.status(200).send("Data saved successfully");
    }
  });
});

// Create HTTPS server
const httpsServer = https.createServer(credentials, app);

httpsServer.listen(PORT, () => {
  console.log(`Server running on https://localhost:${PORT}`);
});
