// Chart instance reference
let perfChart = null;
const MAX_CHART_POINTS = 30;
const latencyHistory = [];
const fpsHistory = [];
const labelsHistory = [];

// Initialize Dashboard when DOM loads
document.addEventListener("DOMContentLoaded", () => {
    initChart();
    setupWebSocket();
    loadSystemConfig();
    setupControlListeners();
    checkStreamActive();
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
    const ctx = document.getElementById('performance-chart').getContext('2d');
    
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

// Periodically check if stream image is loading
function checkStreamActive() {
    const streamImg = document.getElementById("mjpeg-stream");
    streamImg.onerror = () => {
        console.warn("MJPEG stream link failed or offline. Retrying stream reload...");
        setTimeout(() => {
            streamImg.src = "/api/stream?t=" + new Date().getTime();
        }, 5000);
    };
}
