// Chart instance reference
let perfChart = null;
const MAX_CHART_POINTS = 30;
const latencyHistory = [];
const fpsHistory = [];
const labelsHistory = [];

// Initialize Dashboard when DOM loads
document.addEventListener("DOMContentLoaded", () => {
    const initSteps = [
        { name: "initChart", fn: initChart },
        { name: "setupWebSocket", fn: setupWebSocket },
        { name: "loadSystemConfig", fn: loadSystemConfig },
        { name: "setupControlListeners", fn: setupControlListeners },
        { name: "checkStreamActive", fn: checkStreamActive },
        { name: "initViewToggle", fn: initViewToggle },
        { name: "initCalibration", fn: initCalibration }
    ];

    initSteps.forEach(step => {
        try {
            step.fn();
        } catch (e) {
            console.error(`Initialization step failed: ${step.name}`, e);
        }
    });
});

// Load startup config from backend
function loadSystemConfig() {
    fetch('/api/config')
        .then(res => res.json())
        .then(config => {
            if (config && Object.keys(config).length > 0) {
                console.log("Loaded system config:", config);
                
                // Update sliders
                if (config.prob_threshold !== undefined) {
                    document.getElementById("slider-prob").value = config.prob_threshold;
                    document.getElementById("val-prob").innerText = config.prob_threshold.toFixed(2);
                }
                if (config.nms_threshold !== undefined) {
                    document.getElementById("slider-nms").value = config.nms_threshold;
                    document.getElementById("val-nms").innerText = config.nms_threshold.toFixed(2);
                }
                
                // Update mode
                if (config.mode !== undefined) {
                    const selectMode = document.getElementById("select-mode");
                    selectMode.value = config.mode;
                    toggleVideoSourceGroup(config.mode);
                    
                    const activeSourceDisplay = document.getElementById("active-source-display");
                    if (config.mode === "camera") {
                        activeSourceDisplay.innerText = config.camera_device || "/dev/video_source";
                    } else if (config.video_path) {
                        const videoName = config.video_path.split('/').pop();
                        activeSourceDisplay.innerText = videoName;
                        document.getElementById("select-video").value = videoName;
                    }
                }
            }
        })
        .catch(err => console.error("Error loading system config:", err));
}

function toggleVideoSourceGroup(mode) {
    const videoGroup = document.getElementById("video-source-group");
    if (mode === "camera") {
        videoGroup.style.opacity = "0.5";
        videoGroup.style.pointerEvents = "none";
    } else {
        videoGroup.style.opacity = "1";
        videoGroup.style.pointerEvents = "auto";
    }
}

// Setup Chart.js
function initChart() {
    const canvas = document.getElementById('performance-chart');
    if (!canvas) return;

    if (typeof Chart === 'undefined') {
        console.warn("Chart.js is not loaded. Skipping chart initialization.");
        return;
    }

    const ctx = canvas.getContext('2d');
    
    // Fill empty history
    for (let i = 0; i < MAX_CHART_POINTS; i++) {
        latencyHistory.push(0);
        fpsHistory.push(0);
        labelsHistory.push('');
    }

    perfChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labelsHistory,
            datasets: [
                {
                    label: 'Inference Latency (ms)',
                    data: latencyHistory,
                    borderColor: '#8a4bf3',
                    backgroundColor: 'rgba(138, 75, 243, 0.1)',
                    borderWidth: 2,
                    tension: 0.4,
                    fill: true,
                    yAxisID: 'y'
                },
                {
                    label: 'FPS',
                    data: fpsHistory,
                    borderColor: '#00f2fe',
                    backgroundColor: 'rgba(0, 242, 254, 0.05)',
                    borderWidth: 2,
                    tension: 0.4,
                    fill: false,
                    yAxisID: 'y1'
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
                duration: 200 // Faster updates
            },
            scales: {
                x: {
                    grid: {
                        color: 'rgba(255, 255, 255, 0.03)'
                    },
                    ticks: {
                        display: false
                    }
                },
                y: {
                    type: 'linear',
                    display: true,
                    position: 'left',
                    grid: {
                        color: 'rgba(255, 255, 255, 0.05)'
                    },
                    ticks: {
                        color: '#a49fc6'
                    },
                    title: {
                        display: true,
                        text: 'Latency (ms)',
                        color: '#a49fc6'
                    },
                    min: 0,
                    max: 100 // Average latency on Pi 5 is ~20ms, so 100ms is a good bounds
                },
                y1: {
                    type: 'linear',
                    display: true,
                    position: 'right',
                    grid: {
                        drawOnChartArea: false // only want one grid line set
                    },
                    ticks: {
                        color: '#a49fc6'
                    },
                    title: {
                        display: true,
                        text: 'FPS',
                        color: '#a49fc6'
                    },
                    min: 0,
                    max: 75
                }
            },
            plugins: {
                legend: {
                    labels: {
                        color: '#ffffff',
                        font: {
                            family: 'Outfit'
                        }
                    }
                }
            }
        }
    });
}

// Setup WebSocket Connection
function setupWebSocket() {
    const wsScheme = window.location.protocol === "https:" ? "wss" : "ws";
    const wsUrl = `${wsScheme}://${window.location.host}/ws`;
    
    console.log(`Connecting to WebSocket: ${wsUrl}`);
    const socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        console.log("WebSocket connection established.");
        document.getElementById("sys-status").innerText = "ONLINE";
        document.getElementById("sys-status").className = "status-value success";
    };

    socket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            updateDashboard(data);
        } catch (e) {
            console.error("Error parsing WebSocket message:", e);
        }
    };

    socket.onclose = () => {
        console.warn("WebSocket connection closed. Reconnecting in 3 seconds...");
        document.getElementById("sys-status").innerText = "OFFLINE";
        document.getElementById("sys-status").className = "status-value";
        setStreamState(false);
        setTimeout(setupWebSocket, 3000);
    };

    socket.onerror = (error) => {
        console.error("WebSocket error:", error);
    };
}

// Update DOM elements with new telemetry data
function updateDashboard(data) {
    // Hide loading overlay once we get real data
    const loader = document.getElementById("stream-loader");
    if (loader && loader.style.display !== "none") {
        loader.style.opacity = 0;
        setTimeout(() => { loader.style.display = "none"; }, 500);
    }

    // Set Live/Streaming active badge
    setStreamState(data.streaming);

    // Update performance numbers
    document.getElementById("val-inf-latency").innerHTML = `${data.inference_latency_ms.toFixed(1)} <span class="unit">ms</span>`;
    document.getElementById("val-full-latency").innerHTML = `${data.full_latency_ms.toFixed(1)} <span class="unit">ms</span>`;
    document.getElementById("val-fps").innerHTML = `${data.fps.toFixed(1)} <span class="unit">FPS</span>`;

    // Update object count indicators
    if (data.detections) {
        document.getElementById("count-main-lane").innerText = data.detections["main-lane"] || 0;
        document.getElementById("count-other-lane").innerText = data.detections["other-lane"] || 0;
        document.getElementById("count-vehicle").innerText = data.detections["vehicle"] || 0;
        document.getElementById("count-solid-white").innerText = data.detections["solid-white"] || 0;
        document.getElementById("count-solid-yellow").innerText = data.detections["solid-yellow"] || 0;
        document.getElementById("count-dashed-white").innerText = data.detections["dashed-white"] || 0;
        document.getElementById("count-dashed-yellow").innerText = data.detections["dashed-yellow"] || 0;
        document.getElementById("count-double-solid-white").innerText = data.detections["double-solid-white"] || 0;
        document.getElementById("count-parking-zone").innerText = data.detections["parking-zone"] || 0;
        document.getElementById("count-start").innerText = data.detections["start"] || 0;
        document.getElementById("count-stop-line").innerText = data.detections["stop-line"] || 0;
        document.getElementById("count-turn-lane").innerText = data.detections["turn-lane"] || 0;
    }

    // Update Chart.js datasets
    if (perfChart) {
        // Push new value and shift history
        latencyHistory.push(data.inference_latency_ms);
        latencyHistory.shift();
        
        fpsHistory.push(data.fps);
        fpsHistory.shift();

        // Dynamically adjust scale bounds if FPS or Latency is larger than default
        if (data.inference_latency_ms > perfChart.options.scales.y.max) {
            perfChart.options.scales.y.max = Math.ceil(data.inference_latency_ms * 1.2 / 10) * 10;
        }
        if (data.fps > perfChart.options.scales.y1.max) {
            perfChart.options.scales.y1.max = Math.ceil(data.fps * 1.2 / 5) * 5;
        }

        perfChart.update('none'); // Update without full animation for performance
    }
    drawBEV(data);
}

// Toggle Stream state UI indicators
function setStreamState(isStreaming) {
    const badge = document.getElementById("stream-active-badge");
    if (isStreaming) {
        badge.className = "stream-badge";
        badge.querySelector(".badge-text").innerText = "LIVE";
    } else {
        badge.className = "stream-badge idle";
        badge.querySelector(".badge-text").innerText = "IDLE";
    }
}

// Setup input control sliders & dropdown listeners
function setupControlListeners() {
    const sliderProb = document.getElementById("slider-prob");
    const valProb = document.getElementById("val-prob");
    sliderProb.addEventListener("input", (e) => {
        valProb.innerText = parseFloat(e.target.value).toFixed(2);
    });

    const sliderNms = document.getElementById("slider-nms");
    const valNms = document.getElementById("val-nms");
    sliderNms.addEventListener("input", (e) => {
        valNms.innerText = parseFloat(e.target.value).toFixed(2);
    });

    // Apply Settings button click
    const btnApply = document.getElementById("btn-apply");
    btnApply.addEventListener("click", () => {
        const prob = sliderProb.value;
        const nms = sliderNms.value;
        
        btnApply.innerText = "Applying...";
        btnApply.disabled = true;

        fetch(`/api/settings?prob_threshold=${prob}&nms_threshold=${nms}`, { method: 'POST' })
            .then(res => res.json())
            .then(data => {
                if (data.status === "success") {
                    btnApply.innerText = "Applied ✓";
                    setTimeout(() => {
                        btnApply.innerText = "Apply Settings";
                        btnApply.disabled = false;
                    }, 1500);
                } else {
                    alert(`Failed to apply settings: ${data.message}`);
                    btnApply.innerText = "Apply Settings";
                    btnApply.disabled = false;
                }
            })
            .catch(err => {
                console.error("Error setting parameters:", err);
                btnApply.innerText = "Apply Settings";
                btnApply.disabled = false;
            });
    });

    // Change Run Mode selector dropdown
    const selectMode = document.getElementById("select-mode");
    const activeSourceDisplay = document.getElementById("active-source-display");
    selectMode.addEventListener("change", (e) => {
        const mode = e.target.value;
        
        // Show loader again briefly to wait for reload
        const loader = document.getElementById("stream-loader");
        if (loader) {
            loader.style.display = "flex";
            loader.style.opacity = 1;
        }

        toggleVideoSourceGroup(mode);

        fetch(`/api/mode?mode=${mode}`, { method: 'POST' })
            .then(res => res.json())
            .then(data => {
                if (data.status === "success") {
                    console.log(`Run mode changed to: ${mode}`);
                    if (mode === "camera") {
                        activeSourceDisplay.innerText = "/dev/video_source";
                    } else {
                        const selectVideo = document.getElementById("select-video");
                        activeSourceDisplay.innerText = selectVideo.value;
                    }
                } else {
                    alert(`Failed to change run mode: ${data.message}`);
                    if (loader) loader.style.display = "none";
                }
            })
            .catch(err => {
                console.error("Error changing run mode:", err);
                if (loader) loader.style.display = "none";
            });
    });

    // Change Video source selector dropdown
    const selectVideo = document.getElementById("select-video");
    selectVideo.addEventListener("change", (e) => {
        const videoName = e.target.value;
        
        // Show loader again briefly to wait for reload
        const loader = document.getElementById("stream-loader");
        if (loader) {
            loader.style.display = "flex";
            loader.style.opacity = 1;
        }

        fetch(`/api/source?video_name=${videoName}`, { method: 'POST' })
            .then(res => res.json())
            .then(data => {
                if (data.status === "success") {
                    activeSourceDisplay.innerText = videoName;
                    console.log(`Video source changed to: ${videoName}`);
                } else {
                    alert(`Failed to load video source: ${data.message}`);
                    if (loader) loader.style.display = "none";
                }
            })
            .catch(err => {
                console.error("Error changing video source:", err);
                if (loader) loader.style.display = "none";
            });
    });
}

let currentViewMode = "normal";

// Periodically check if stream image is loading
function checkStreamActive() {
    const streamImg = document.getElementById("mjpeg-stream");
    streamImg.onerror = () => {
        console.warn("MJPEG stream link failed or offline. Retrying stream reload...");
        setTimeout(() => {
            streamImg.src = `/api/stream?view=${currentViewMode}&t=` + new Date().getTime();
        }, 5000);
    };
}

function initViewToggle() {
    const btnNormal = document.getElementById("btn-view-normal");
    const btnIpm = document.getElementById("btn-view-ipm");
    const streamImg = document.getElementById("mjpeg-stream");

    if (!btnNormal || !btnIpm || !streamImg) return;

    btnNormal.addEventListener("click", () => {
        if (currentViewMode === "normal") return;
        currentViewMode = "normal";
        btnNormal.classList.add("active");
        btnIpm.classList.remove("active");
        streamImg.src = "/api/stream?view=normal";
    });

    btnIpm.addEventListener("click", () => {
        if (currentViewMode === "ipm") return;
        currentViewMode = "ipm";
        btnIpm.classList.add("active");
        btnNormal.classList.remove("active");
        streamImg.src = "/api/stream?view=ipm";
    });
}

// --- CALIBRATION & BEV INTEGRATION ---

// Calibration State
let calibrationPoints = []; // Stores up to 4 points: { u, v }
let activeCalibration = null;
const POINT_COLORS = ['#ff007f', '#00f2fe', '#ffff00', '#00ff66'];

function initCalibration() {
    const btnCalibrate = document.getElementById("btn-calibrate");
    const modal = document.getElementById("calibration-modal");
    const btnClose = document.getElementById("btn-close-calibration");
    const canvas = document.getElementById("calibration-canvas");
    const img = document.getElementById("calibration-img");
    const btnSave = document.getElementById("btn-save-calibration");
    const btnClear = document.getElementById("btn-clear-calibration");

    if (!btnCalibrate || !modal) return;

    // Update canvas size when image loads
    img.addEventListener("load", () => {
        canvas.width = img.naturalWidth || 640;
        canvas.height = img.naturalHeight || 480;
        drawCalibrationPoints();
    });

    // Open Modal
    btnCalibrate.addEventListener("click", () => {
        // Fetch snapshot frame
        img.src = `/api/calibration/frame?t=${Date.now()}`;
        modal.classList.remove("hidden");
        calibrationPoints = [];
        drawCalibrationPoints();
        
        // Fetch existing calibration to populate inputs
        fetch('/api/calibration')
            .then(res => res.json())
            .then(data => {
                if (data && data.status !== "error") {
                    activeCalibration = data;
                    document.getElementById("bev-status").innerText = "CALIBRATED";
                    document.getElementById("bev-status").className = "badge success";
                    
                    // Pre-populate input fields
                    if (data.world_points) {
                        for (let i = 0; i < 4; i++) {
                            if (data.world_points[i]) {
                                document.getElementById(`p${i+1}-x`).value = data.world_points[i][0];
                                document.getElementById(`p${i+1}-y`).value = data.world_points[i][1];
                            }
                        }
                    }
                    
                    // Pre-populate pixel points
                    if (data.pixel_points) {
                        calibrationPoints = data.pixel_points.map(pt => ({ u: pt[0], v: pt[1] }));
                        drawCalibrationPoints();
                    }
                }
            })
            .catch(err => console.warn("No active calibration loaded:", err));
    });

    // Close Modal
    btnClose.addEventListener("click", () => {
        modal.classList.add("hidden");
    });

    // Handle clicks on canvas
    canvas.addEventListener("click", (e) => {
        if (calibrationPoints.length >= 4) return;

        const rect = canvas.getBoundingClientRect();
        
        // Use natural dimensions of the loaded image
        const imgW = img.naturalWidth || 640;
        const imgH = img.naturalHeight || 480;
        const u = Math.round((e.clientX - rect.left) * (imgW / rect.width));
        const v = Math.round((e.clientY - rect.top) * (imgH / rect.height));

        calibrationPoints.push({ u, v });
        drawCalibrationPoints();
    });

    // Clear calibration points
    btnClear.addEventListener("click", () => {
        calibrationPoints = [];
        drawCalibrationPoints();
    });

    // Save calibration API request
    btnSave.addEventListener("click", () => {
        if (calibrationPoints.length !== 4) {
            alert("Please select exactly 4 points on the image first.");
            return;
        }

        const worldPoints = [];
        for (let i = 1; i <= 4; i++) {
            const xVal = parseFloat(document.getElementById(`p${i}-x`).value);
            const yVal = parseFloat(document.getElementById(`p${i}-y`).value);
            if (isNaN(xVal) || isNaN(yVal)) {
                alert(`Please enter valid X and Y coordinates for Point ${i}.`);
                return;
            }
            worldPoints.push([xVal, yVal]);
        }

        const payload = {
            pixel_points: calibrationPoints.map(pt => [pt.u, pt.v]),
            world_points: worldPoints,
            image_size: [img.naturalWidth || 640, img.naturalHeight || 480]
        };

        btnSave.innerText = "Saving...";
        btnSave.disabled = true;

        fetch('/api/calibration', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        })
        .then(res => res.json())
        .then(data => {
            if (data.status === "success") {
                alert("Calibration saved successfully!");
                activeCalibration = data;
                document.getElementById("bev-status").innerText = "CALIBRATED";
                document.getElementById("bev-status").className = "badge success";
                modal.classList.add("hidden");
            } else {
                alert("Failed to save calibration: " + data.message);
            }
        })
        .catch(err => {
            console.error("Calibration error:", err);
            alert("Error sending calibration data to server.");
        })
        .finally(() => {
            btnSave.innerText = "Save Calibration";
            btnSave.disabled = false;
        });
    });
    
    // Initial load of calibration status on startup
    fetch('/api/calibration')
        .then(res => res.json())
        .then(data => {
            if (data && data.status !== "error") {
                activeCalibration = data;
                document.getElementById("bev-status").innerText = "CALIBRATED";
                document.getElementById("bev-status").className = "badge success";
                drawBEV(null); // Draw empty grid
            }
        })
        .catch(() => {});
}

function drawCalibrationPoints() {
    const canvas = document.getElementById("calibration-canvas");
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw lines connecting points if we have points
    if (calibrationPoints.length > 0) {
        ctx.beginPath();
        ctx.moveTo(calibrationPoints[0].u, calibrationPoints[0].v);
        for (let i = 1; i < calibrationPoints.length; i++) {
            ctx.lineTo(calibrationPoints[i].u, calibrationPoints[i].v);
        }
        if (calibrationPoints.length === 4) {
            ctx.closePath();
            ctx.strokeStyle = "rgba(138, 75, 243, 0.6)";
            ctx.lineWidth = 3;
            ctx.stroke();
            ctx.fillStyle = "rgba(138, 75, 243, 0.15)";
            ctx.fill();
        } else {
            ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
            ctx.lineWidth = 2;
            ctx.stroke();
        }
    }

    // Draw point markers
    calibrationPoints.forEach((pt, idx) => {
        const color = POINT_COLORS[idx];
        
        // Glow effect
        ctx.shadowBlur = 8;
        ctx.shadowColor = color;
        
        // Outer dot
        ctx.beginPath();
        ctx.arc(pt.u, pt.v, 8, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();
        
        // Reset shadow
        ctx.shadowBlur = 0;

        // Label text
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 12px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(idx + 1, pt.u, pt.v);
    });
}

// Color palette matching C++ for classes (RGBA formats for canvas)
const CLASS_COLORS_RGBA = [
    'rgba(51, 102, 255, 0.8)',   // dashed-white
    'rgba(255, 153, 0, 0.8)',   // dashed-yellow
    'rgba(0, 119, 255, 0.8)',   // double-solid-white
    'rgba(0, 255, 102, 0.8)',   // main-lane
    'rgba(255, 51, 51, 0.8)',   // other-lane
    'rgba(128, 128, 128, 0.8)', // parking-zone
    'rgba(0, 242, 254, 0.8)',   // solid-white
    'rgba(255, 255, 0, 0.8)',   // solid-yellow
    'rgba(0, 255, 127, 0.8)',   // start
    'rgba(128, 0, 0, 0.8)',     // stop-line
    'rgba(170, 0, 255, 0.8)',   // turn-lane
    'rgba(255, 0, 255, 0.8)'    // vehicle
];

function drawBEV(telemetry) {
    const canvas = document.getElementById("bev-canvas");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    
    const w = canvas.width;
    const h = canvas.height;
    
    // Clear canvas
    ctx.clearRect(0, 0, w, h);
    
    // Coordinate mapping bounds
    // Lateral range (X): -1000mm to +1000mm (width 2000mm)
    // Longitudinal range (Y): 0mm to 3500mm (height 3500mm)
    const xRange = 2000;
    const yRange = 3500;
    
    const scaleX = w / xRange;
    const scaleY = h / yRange;
    
    const toCanvasX = (X) => w / 2 + X * scaleX;
    const toCanvasY = (Y) => h - Y * scaleY;

    // Draw Grid Lines (every 500mm)
    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 1;
    
    // Horizontal lines (Y distance)
    for (let y = 500; y <= yRange; y += 500) {
        const cy = toCanvasY(y);
        ctx.beginPath();
        ctx.moveTo(0, cy);
        ctx.lineTo(w, cy);
        ctx.stroke();
        
        ctx.fillStyle = "rgba(164, 159, 198, 0.4)";
        ctx.font = "9px monospace";
        ctx.textAlign = "left";
        ctx.fillText(`${(y/1000).toFixed(1)}m`, 5, cy - 3);
    }
    
    // Vertical lines (X lateral offset)
    for (let x = -1000; x <= 1000; x += 500) {
        const cx = toCanvasX(x);
        ctx.beginPath();
        ctx.moveTo(cx, 0);
        ctx.lineTo(cx, h);
        ctx.stroke();
        
        if (x !== 0) {
            ctx.fillStyle = "rgba(164, 159, 198, 0.4)";
            ctx.font = "9px monospace";
            ctx.textAlign = "center";
            ctx.fillText(`${x > 0 ? '+' : ''}${(x/1000).toFixed(1)}m`, cx, h - 5);
        }
    }

    // Draw centerline (Y axis)
    ctx.strokeStyle = "rgba(138, 75, 243, 0.2)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(toCanvasX(0), 0);
    ctx.lineTo(toCanvasX(0), h);
    ctx.stroke();

    // Draw vehicle shape at bottom center (X=0, Y=0)
    const vWidth = 200 * scaleX;
    const vHeight = 300 * scaleY;
    
    ctx.fillStyle = "rgba(0, 242, 254, 0.3)";
    ctx.strokeStyle = "var(--accent-cyan)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(w / 2 - vWidth / 2, h - vHeight, vWidth, vHeight);
    ctx.fill();
    ctx.stroke();
    
    // Center point representing origin
    ctx.fillStyle = "var(--accent-cyan)";
    ctx.beginPath();
    ctx.arc(w / 2, h, 4, 0, 2 * Math.PI);
    ctx.fill();

    // Draw transformed polygons from telemetry
    if (telemetry && telemetry.objects) {
        let hasRealWorldCoords = false;
        
        telemetry.objects.forEach(obj => {
            const color = CLASS_COLORS_RGBA[obj.label % CLASS_COLORS_RGBA.length];

            if (obj.polygons_real_world && obj.polygons_real_world.length > 0) {
                hasRealWorldCoords = true;
                
                ctx.fillStyle = color;
                ctx.strokeStyle = color.replace('0.8', '1.0');
                ctx.lineWidth = 2;
                
                obj.polygons_real_world.forEach(poly => {
                    if (poly.length === 0) return;
                    
                    ctx.beginPath();
                    ctx.moveTo(toCanvasX(poly[0][0]), toCanvasY(poly[0][1]));
                    for (let i = 1; i < poly.length; i++) {
                        ctx.lineTo(toCanvasX(poly[i][0]), toCanvasY(poly[i][1]));
                    }
                    ctx.closePath();
                    ctx.fill();
                    ctx.stroke();
                });
            }

            // Draw waypoints
            if (obj.waypoints && obj.waypoints.length > 0) {
                obj.waypoints.forEach(wp => {
                    ctx.fillStyle = '#ffffff';
                    ctx.strokeStyle = color.replace('0.8', '1.0');
                    ctx.lineWidth = 1.5;
                    ctx.beginPath();
                    ctx.arc(toCanvasX(wp[0]), toCanvasY(wp[1]), 3, 0, 2 * Math.PI);
                    ctx.fill();
                    ctx.stroke();
                });
            }

            // Draw fitted polynomial curve
            if (obj.polynomial) {
                const poly = obj.polynomial;
                const a3 = poly.a3 || 0;
                const a2 = poly.a2 || 0;
                const a1 = poly.a1 || 0;
                const a0 = poly.a0 || 0;

                if (a3 !== 0 || a2 !== 0 || a1 !== 0 || a0 !== 0) {
                    ctx.strokeStyle = color.replace('0.8', '1.0');
                    ctx.lineWidth = 3;
                    ctx.beginPath();
                    let first = true;

                    if (obj.label === 10) {  // turn-lane: fitted as y(x)
                        // Sweep X from -1000 to 1000 in steps of 50
                        for (let x_val = -1000; x_val <= 1000; x_val += 50) {
                            const y_val = a3 * Math.pow(x_val, 3) + a2 * Math.pow(x_val, 2) + a1 * x_val + a0;
                            const cx = toCanvasX(x_val);
                            const cy = toCanvasY(y_val);
                            if (cx >= 0 && cx <= w && cy >= 0 && cy <= h) {
                                if (first) {
                                    ctx.moveTo(cx, cy);
                                    first = false;
                                } else {
                                    ctx.lineTo(cx, cy);
                                }
                            }
                        }
                    } else {  // regular lanes: fitted as x(y)
                        // Sweep Y from 0 to 3500 in steps of 100
                        for (let y_val = 0; y_val <= 3500; y_val += 100) {
                            const x_val = a3 * Math.pow(y_val, 3) + a2 * Math.pow(y_val, 2) + a1 * y_val + a0;
                            const cx = toCanvasX(x_val);
                            const cy = toCanvasY(y_val);
                            if (cx >= 0 && cx <= w && cy >= 0 && cy <= h) {
                                if (first) {
                                    ctx.moveTo(cx, cy);
                                    first = false;
                                } else {
                                    ctx.lineTo(cx, cy);
                                }
                            }
                        }
                    }
                    ctx.stroke();
                }
            }
        });
        
        // Update status badge if real-world telemetry is received
        const statusBadge = document.getElementById("bev-status");
        if (hasRealWorldCoords && statusBadge) {
            statusBadge.innerText = "CALIBRATED";
            statusBadge.className = "badge success";
            statusBadge.classList.remove("idle");
        }
    }
}
