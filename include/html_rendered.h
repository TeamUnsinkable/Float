const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML>
<html>
  <head>
    <title>ESP32-Driven Float</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">

    <style>
      * {
        box-sizing: border-box;
      }

      html,
      body {
        min-height: 100%;
      }

      body {
        margin: 0;
        padding: 16px;
        font-family: Arial, sans-serif;
        background: #f4f4f4;
        text-align: center;

        display: flex;
        justify-content: center;
        align-items: center;
      }

      .page-container {
        width: 100%;
        max-width: 900px;
      }

      .main-panel {
        width: 100%;
        padding: 20px;
        border: 1px solid #888;
        border-radius: 12px;
        background: white;
      }

      h1 {
        margin: 0 0 22px 0;
        font-size: 1.7rem;
      }

      .button {
        width: 100%;
        max-width: 260px;
        padding: 14px 20px;
        font-size: 24px;
        text-align: center;
        outline: none;
        color: #fff;
        background-color: #2f4468;
        border: none;
        border-radius: 8px;
        box-shadow: 0 6px #999;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
      }

      .button:hover {
        background-color: #1f2e45;
      }

      .button:active {
        background-color: #1f2e45;
        box-shadow: 0 4px #666;
        transform: translateY(2px);
      }

      .tuning-panel,
      .readings-panel {
        width: 100%;
        margin-top: 28px;
        padding-top: 22px;
        border-top: 1px solid #ddd;
      }

      .tuning-summary,
      .readings-panel h2 {
        margin: 0 0 18px 0;
        text-align: center;
        font-size: 1.35rem;
        font-weight: bold;
      }

      .tuning-summary {
        cursor: pointer;
        list-style: none;
        padding: 12px;
        border-radius: 8px;
        background: #e9e9e9;
        border: 1px solid #ccc;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
      }

      .tuning-summary::-webkit-details-marker {
        display: none;
      }

      .tuning-summary::after {
        content: " -";
        font-size: 0.9rem;
      }

      .tuning-panel:not([open]) .tuning-summary::after {
        content: " +";
      }

      .tuning-content {
        margin-top: 18px;
      }

      .tuning-row {
        display: grid;
        grid-template-columns: 120px minmax(0, 1fr);
        align-items: center;
        gap: 12px;
        margin-bottom: 12px;
      }

      .tuning-row label {
        font-weight: bold;
        font-size: 1.05rem;
        text-align: left;
      }

      .tuning-row input {
        width: 100%;
        min-width: 0;
        padding: 12px;
        font-size: 1.1rem;
        border: 1px solid #777;
        border-radius: 8px;
      }

      .tuning-buttons {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 12px;
        margin-top: 18px;
      }

      .readings-buttons {
        display: grid;
        grid-template-columns: 1fr;
        gap: 12px;
        margin-top: 18px;
      }

      @media (min-width: 600px) {
        .readings-buttons {
          grid-template-columns: 1fr 1fr 1fr;
        }
      }

      .tuning-buttons button,
      .readings-buttons button {
        width: 100%;
        padding: 14px;
        font-size: 1.1rem;
        border-radius: 8px;
        border: 1px solid #555;
        background: #e9e9e9;
        cursor: pointer;
        -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
      }

      .tuning-buttons button:active,
      .readings-buttons button:active {
        transform: translateY(1px);
      }

      #tuningStatus,
      #tableStatus {
        min-height: 1.4em;
        margin: 14px 0 0 0;
        text-align: center;
        font-size: 0.95rem;
      }

      .table-wrapper {
        width: 100%;
        max-height: 360px;
        margin-top: 12px;
        overflow: auto;
        border: 1px solid #ddd;
        border-radius: 8px;
      }

      #readingsTable {
        width: 100%;
        min-width: 620px;
        border-collapse: collapse;
        font-size: 0.85rem;
      }

      #readingsTable th,
      #readingsTable td {
        padding: 8px;
        border-bottom: 1px solid #ddd;
        text-align: center;
        white-space: nowrap;
      }

      #readingsTable th {
        position: sticky;
        top: 0;
        background: #f0f0f0;
        font-weight: bold;
        z-index: 1;
      }

      #readingsTable tbody tr:last-child td {
        border-bottom: none;
      }

      .log-wrapper {
        margin-top: 12px;
      }

      #readingsLog {
        text-align: left;
        background: #f7f7f7;
        border: 1px solid #ddd;
        border-radius: 8px;
        padding: 10px;
        max-height: 400px;
        overflow-y: auto;
        white-space: pre-wrap;
        word-break: break-word;
        font-size: 0.85rem;
      }

      @media (max-width: 430px) {
        body {
          padding: 10px;
        }

        .main-panel {
          padding: 14px;
        }

        h1 {
          font-size: 1.45rem;
        }

        .button {
          font-size: 21px;
        }

        .tuning-row {
          grid-template-columns: 92px minmax(0, 1fr);
          gap: 8px;
        }

        .tuning-row label {
          font-size: 0.95rem;
        }

        .tuning-row input {
          padding: 10px;
          font-size: 1rem;
        }

        .tuning-buttons,
        .readings-buttons {
          gap: 8px;
        }

        .tuning-buttons button,
        .readings-buttons button {
          padding: 12px;
          font-size: 1rem;
        }

        #readingsTable {
          font-size: 0.8rem;
        }
      }
    </style>
  </head>

  <body>
    <main class="page-container">
      <section class="main-panel">
        <h1>ESP32-Driven Float</h1>

        <button
          class="button"
          onmousedown="toggleCheckbox('on');"
          ontouchstart="toggleCheckbox('on');"
          onmouseup="toggleCheckbox('off');"
          ontouchend="toggleCheckbox('off');">
          DIVE!
        </button>

        <details class="tuning-panel" open>
          <summary class="tuning-summary">Controller Tuning</summary>

          <div class="tuning-content">
            <div class="tuning-row">
              <label for="kiInput">Ki</label>
              <input type="number" id="kiInput" step="0.0001" inputmode="decimal" value="0.0">
            </div>

            <div class="tuning-row">
              <label for="kpInput">Kp</label>
              <input type="number" id="kpInput" step="0.0001" inputmode="decimal" value="0.0">
            </div>

            <div class="tuning-row">
              <label for="kdInput">Kd</label>
              <input type="number" id="kdInput" step="0.0001" inputmode="decimal" value="0.0">
            </div>

            <div class="tuning-row">
              <label for="setpointInput">Setpoint</label>
              <input type="number" id="setpointInput" step="0.001" inputmode="decimal" value="0.0">
            </div>

            <div class="tuning-row">
              <label for="loopTimeInput">Loop ms</label>
              <input type="number" id="loopTimeInput" step="1" min="1" inputmode="numeric" value="100">
            </div>

            <div class="tuning-buttons">
              <button type="button" onclick="getTuning()">Get</button>
              <button type="button" onclick="pushTuning()">Push</button>
            </div>

            <p id="tuningStatus">Ready.</p>
          </div>
        </details>

        <div class="readings-panel">
          <h2>Depth Readings</h2>

          <div class="readings-buttons">
            <button type="button" onclick="getReadingsTable()">View Table</button>
            <button type="button" onclick="downloadReadingsCSV()">Download CSV</button>
            <button type="button" onclick="getReadingsLog()">View Log</button>
          </div>

          <p id="tableStatus">No table loaded.</p>

          <div class="table-wrapper">
            <table id="readingsTable">
              <thead>
                <tr>
                  <th>Run</th>
                  <th>Time</th>
                  <th>Depth Pa</th>
                  <th>Depth m</th>
                  <th>Packet</th>
                </tr>
              </thead>
              <tbody id="readingsTableBody">
              </tbody>
            </table>
          </div>

          <div class="log-wrapper">
            <pre id="readingsLog"></pre>
          </div>
        </div>
      </section>
    </main>

    <script>
      function toggleCheckbox(x) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/" + x, true);
        xhr.send();
      }

      function getTuning() {
        fetch("/getTuning")
          .then(response => response.json())
          .then(data => {
            document.getElementById("kiInput").value = data.ki;
            document.getElementById("kpInput").value = data.kp;
            document.getElementById("kdInput").value = data.kd;
            document.getElementById("setpointInput").value = data.setpoint;
            document.getElementById("loopTimeInput").value = data.loop_ms;

            document.getElementById("tuningStatus").innerText =
              "Tuning values loaded.";
          })
          .catch(error => {
            document.getElementById("tuningStatus").innerText =
              "Failed to get tuning values.";
          });
      }

      function pushTuning() {
        const ki = parseFloat(document.getElementById("kiInput").value);
        const kp = parseFloat(document.getElementById("kpInput").value);
        const kd = parseFloat(document.getElementById("kdInput").value);
        const setpoint = parseFloat(document.getElementById("setpointInput").value);
        const loopMs = parseInt(document.getElementById("loopTimeInput").value, 10);

        if (
          isNaN(ki) ||
          isNaN(kp) ||
          isNaN(kd) ||
          isNaN(setpoint) ||
          isNaN(loopMs) ||
          loopMs < 1
        ) {
          document.getElementById("tuningStatus").innerText =
            "Enter valid tuning values.";
          return;
        }

        const url =
          "/updateTuning" +
          "?ki=" + encodeURIComponent(ki) +
          "&kp=" + encodeURIComponent(kp) +
          "&kd=" + encodeURIComponent(kd) +
          "&setpoint=" + encodeURIComponent(setpoint) +
          "&loop_ms=" + encodeURIComponent(loopMs);

        fetch(url)
          .then(response => response.text())
          .then(data => {
            document.getElementById("tuningStatus").innerText = data;
          })
          .catch(error => {
            document.getElementById("tuningStatus").innerText =
              "Failed to push tuning values.";
          });
      }

      function getReadingsTable() {
        document.getElementById("tableStatus").innerText = "Loading CSV...";
        document.getElementById("readingsTableBody").innerHTML = "";

        fetch("/getReadingsCSV")
          .then(response => response.text())
          .then(csvText => {
            const tableBody = document.getElementById("readingsTableBody");
            tableBody.innerHTML = "";

            const lines = csvText.trim().split(/\r?\n/);

            if (lines.length <= 1) {
              document.getElementById("tableStatus").innerText =
                "No readings available.";
              return;
            }

            let loadedRows = 0;

            for (let i = 1; i < lines.length; i++) {
              const cols = lines[i].split(",");

              if (cols.length < 7) {
                continue;
              }

              const runNumber = cols[0];
              const lHour = cols[1];
              const lMin = cols[2];
              const lSec = cols[3];
              const depthPa = cols[4];
              const depthM = cols[5];
              const packet = cols[6];

              const timeString =
                String(lHour).padStart(2, "0") + ":" +
                String(lMin).padStart(2, "0") + ":" +
                String(lSec).padStart(2, "0");

              const tr = document.createElement("tr");

              tr.innerHTML =
                "<td>" + runNumber + "</td>" +
                "<td>" + timeString + "</td>" +
                "<td>" + depthPa + "</td>" +
                "<td>" + depthM + "</td>" +
                "<td>" + packet + "</td>";

              tableBody.appendChild(tr);
              loadedRows++;
            }

            document.getElementById("tableStatus").innerText =
              "Loaded " + loadedRows + " readings.";
          })
          .catch(error => {
            document.getElementById("tableStatus").innerText =
              "Failed to load CSV.";
          });
      }

      function downloadReadingsCSV() {
        window.location.href = "/getReadingsCSV";
      }

      function getReadingsLog() {
        document.getElementById("tableStatus").innerText = "Loading log...";

        fetch("/getReadingsCSV")
          .then(response => response.text())
          .then(csvText => {
            const lines = csvText.trim().split(/\r?\n/);

            if (lines.length <= 1) {
              document.getElementById("tableStatus").innerText =
                "No readings available.";
              return;
            }

            let output = "";
            let loadedRows = 0;

            for (let i = 1; i < lines.length; i++) {
              const cols = lines[i].split(",");

              if (cols.length < 7) {
                continue;
              }

              output +=
                "Profile#: " + cols[0] +
                "  EX01  " +
                cols[1] + ":" + cols[2] + ":" + cols[3] +
                "  EST   " +
                cols[4] +
                " kPa  " +
                cols[5] +
                " meters\n";

              loadedRows++;
            }

            document.getElementById("readingsLog").textContent = output;

            document.getElementById("tableStatus").innerText =
              "Loaded " + loadedRows + " readings.";
          })
          .catch(error => {
            document.getElementById("tableStatus").innerText =
              "Failed to load log.";
          });
      }
    </script>
  </body>
</html>)rawliteral";