// ============================================================
// InfluxDB Mapper — Prep CAN data for Stackhero
// ============================================================

const decoded = msg.payload;

// Don't write unknown frames to the DB
if (decoded.sensor === "unknown" || decoded.sensor === "dummy") {
    return null;
}

// 1. Define the Measurement name
const measurementName = "can_sensor_data";

// 2. Prepare Fields (the numerical values we want to graph)
let fields = {};

// Handle different sensor types based on the decoded object
if (decoded.degrees !== undefined) {
    fields.value = decoded.degrees;
} else if (decoded.rpm !== undefined) {
    fields.value = decoded.rpm;
} else if (decoded.voltage !== undefined) {
    fields.value = decoded.voltage;
} else if (decoded.x_g !== undefined) {
    // For accelerometers, we store all 3 axes in one point
    fields.x = decoded.x_g;
    fields.y = decoded.y_g;
    fields.z = decoded.z_g;
}

// 3. Construct the Stackhero-formatted payload
msg.payload = {
    bucket: "testBucket", // <--- Make sure this matches your InfluxDB bucket
    precision: "ms",
    data: {
        measurement: measurementName,
        tags: {
            sensor_type: decoded.sensor, // e.g., "ADXL345_1"
            can_id: decoded.frame        // e.g., "0x002"
        },
        fields: fields,
        timestamp: Date.now() // Uses Node-RED's system time
    }
};

return msg;
