const compassSlider = document.getElementById("compassSlider"); // Get the compass slider element from the HTML page

// Check if the compassSlider element exists and reset it to 0
if (compassSlider) {
  const middleValue = 0;
  compassSlider.value = middleValue; // Set slider to the middle position (0 degrees)
  updateCompass(middleValue); // Set the compass value to initial value of 0
}

// Function to update the compass display (show the current compass position)
function updateCompass(pos) {
  document.getElementById("compassValue").innerText = `${pos}°`; // Update the text value for compassValue when value is changing on page element with the current position and add "°" for degrees
}

// Function to send the current compass value to the server
function sendCompassValue(pos) {
  fetch(`/compass?value=${pos}`); // Send the compass value to the server using a fetch request
  console.log("Compass value", pos); // Log the compass value to the console for debugging
}

// Functions for moving the motor forward and backward with specific distances
function forwards5() {
  move("forwards", 5);
} // Calling move function with separated parameters
function forwards20() {
  move("forwards", 20);
} // Calling move function with separated parameters
function backwards5() {
  move("backwards", 5);
} // Calling move function with separated parameters
function backwards20() {
  move("backwards", 20);
} // Calling move function with separated parameters

// This is a general-purpose function that can handle different combinations of direction and distance
function move(dir, dis) {
  fetch(`/${dir}${dis}`); // Sends a request to the server with a URL constructed from the received direction and distance (e.g., /forwards5)
  console.log("Drive", dir, dis); // Log the movement command to the console for debugging
}

// Function to face north
function faceNorth() {
  fetch("/north");
  console.log("face north");
}

// Lidar polling
const lidarValueEl = document.getElementById("lidarValue");

async function pollLidar() {
  try {
    const res = await fetch("/lidar");
    if (!res.ok) return;
    const text = await res.text();
    const trimmed = text.trim();
    // Expected format: LIDAR:<cm>,SEQ:<n>,T:<ms>
    let display = trimmed;
    if (trimmed.includes(",")) {
      const first = trimmed.split(",")[0];
      if (first.startsWith("LIDAR:")) {
        display = first.substring(6);
      }
    } else if (trimmed.startsWith("LIDAR:")) {
      display = trimmed.substring(6);
    }
    if (lidarValueEl) lidarValueEl.innerText = display.trim() + " cm";

    // Console-only stats
    if (trimmed.includes("SEQ:") && trimmed.includes("T:")) {
      const parts = trimmed.split(",");
      const seqStr = parts[1].split(":")[1];
      const tStr = parts[2].split(":")[1];
      const seq = parseInt(seqStr, 10);
      const t = parseInt(tStr, 10);
      if (!isNaN(seq)) {
        if (window._lastSeq !== undefined && seq > window._lastSeq + 1) {
          window._dropCount = (window._dropCount || 0) + (seq - window._lastSeq - 1);
        }
        window._lastSeq = seq;
      }
      if (!isNaN(t)) {
        window._recvCount = (window._recvCount || 0) + 1;
        window._lastTs = t;
      }
    }
  } catch (e) {
    // ignore transient errors
  }
}

setInterval(pollLidar, 20);
pollLidar();

// Console-only summary every 5s
setInterval(() => {
  const recv = window._recvCount || 0;
  const drop = window._dropCount || 0;
  const now = Date.now();
  if (!window._startTs) window._startTs = now;
  const elapsed = (now - window._startTs) / 1000.0;
  if (elapsed <= 0) return;
  const hz = recv / elapsed;
  const dropsPct = (recv + drop) > 0 ? (drop / (recv + drop)) * 100.0 : 0;
  console.log(`WEB rate ${hz.toFixed(1)} Hz, drops ${dropsPct.toFixed(1)}%`);
}, 5000);

// Compass polling
const compassWebValueEl = document.getElementById("compassWebValue");
async function pollCompass() {
  try {
    const res = await fetch("/compassdata");
    if (!res.ok) return;
    const text = await res.text();
    if (compassWebValueEl) {
      const parts = text.trim().split(",");
      const deg = parts[0] || "--";
      const label = parts[1] || "";
      compassWebValueEl.innerText = `${deg}° ${label}`;
    }
  } catch (e) {
    // ignore transient errors
  }
}

setInterval(pollCompass, 1000);
pollCompass();
