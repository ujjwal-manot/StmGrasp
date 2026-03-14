#include "web_server.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

static AsyncWebServer _server(WEB_PORT);
static AsyncWebSocket _ws("/ws");
static WebCommandCallback _cmd_callback = nullptr;

// ─────────────────────────────────────────────────────────────
// Dashboard HTML — entirely self-contained, no CDN dependencies
// ─────────────────────────────────────────────────────────────
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HYDRA Grasp</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0e17;color:#e0e6f0;font-family:'Segoe UI',system-ui,sans-serif;font-size:14px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:8px;padding:8px;max-width:1200px;margin:0 auto}
.panel{background:#141b2d;border:1px solid #1e2a42;border-radius:8px;padding:12px;min-height:180px}
.panel h3{font-size:12px;color:#5a6f8f;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
canvas{width:100%;height:140px;display:block;border-radius:4px}
.hdr{grid-column:1/-1;display:flex;justify-content:space-between;align-items:center;padding:8px 12px;background:#141b2d;border-radius:8px;border:1px solid #1e2a42}
.hdr h1{font-size:18px;font-weight:700;color:#4fc3f7}
.badge{display:inline-block;padding:4px 12px;border-radius:12px;font-weight:600;font-size:13px}
.ctrl{grid-column:1/-1;display:flex;gap:8px;justify-content:center}
.btn{padding:10px 24px;border:none;border-radius:6px;font-size:14px;font-weight:600;cursor:pointer;color:#fff;transition:opacity .2s}
.btn:hover{opacity:.85}
.btn-go{background:#27ae60}.btn-rel{background:#e67e22}.btn-stop{background:#e74c3c}
.qbar{height:8px;background:#1e2a42;border-radius:4px;margin-top:6px;overflow:hidden}
.qfill{height:100%;border-radius:4px;transition:width .3s}
.mat-badge{font-size:22px;font-weight:700;text-align:center;padding:16px 0}
.stat-val{font-size:28px;font-weight:700;text-align:center;padding:8px 0}
.stat-label{font-size:11px;color:#5a6f8f;text-align:center}
.depth-grid{display:grid;grid-template-columns:repeat(8,1fr);gap:1px}
.depth-cell{aspect-ratio:1;border-radius:2px;font-size:7px;display:flex;align-items:center;justify-content:center}
</style>
</head><body>
<div class="grid">
 <div class="hdr"><h1>HYDRA Grasp</h1><span id="state" class="badge" style="background:#2980b9">IDLE</span></div>

 <div class="panel"><h3>Impedance Map</h3><canvas id="cImp"></canvas></div>
 <div class="panel"><h3>Acoustic FFT</h3><canvas id="cFFT"></canvas></div>
 <div class="panel"><h3>Force Distribution</h3><canvas id="cForce"></canvas></div>
 <div class="panel"><h3>Depth Grid (8x8)</h3><div id="depthGrid" class="depth-grid"></div></div>

 <div class="panel"><h3>Curvature</h3><canvas id="cCurv"></canvas></div>
 <div class="panel">
  <h3>State &amp; Quality</h3>
  <div id="stateText" class="stat-val">IDLE</div>
  <div class="stat-label">Grasp Quality</div>
  <div class="qbar"><div id="qFill" class="qfill" style="width:0%;background:#27ae60"></div></div>
  <div id="qVal" class="stat-label">0%</div>
 </div>
 <div class="panel">
  <h3>Material</h3>
  <div id="matBadge" class="mat-badge">--</div>
  <div class="stat-label">Confidence</div>
  <div class="qbar"><div id="matConf" class="qfill" style="width:0%;background:#3498db"></div></div>
  <div id="matConfVal" class="stat-label">0%</div>
 </div>
 <div class="panel">
  <h3>Plan</h3>
  <div style="font-size:12px;line-height:1.8;padding:4px 0">
   <div>Target: <b id="planForce">--</b> N</div>
   <div>Speed: <b id="planSpeed">--</b></div>
   <div>Aperture: <b id="planAper">--</b> mm</div>
   <div>Ramp: <b id="planRamp">--</b> N/s</div>
  </div>
 </div>

 <div class="ctrl">
  <button class="btn btn-go" onclick="ws_cmd('start')">START GRASP</button>
  <button class="btn btn-rel" onclick="ws_cmd('release')">RELEASE</button>
  <button class="btn btn-stop" onclick="ws_cmd('estop')">E-STOP</button>
 </div>
</div>

<script>
var ws,D={};
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onmessage=function(e){
  try{D=JSON.parse(e.data);render();}catch(x){}
 };
 ws.onclose=function(){setTimeout(connect,1000);};
}
connect();
function ws_cmd(c){if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:c}));}

function render(){
 var s=D.state||'IDLE';
 document.getElementById('state').textContent=s;
 document.getElementById('stateText').textContent=s;
 var q=Math.round((D.quality||0)*100);
 document.getElementById('qFill').style.width=q+'%';
 document.getElementById('qFill').style.background=q>60?'#27ae60':q>30?'#f39c12':'#e74c3c';
 document.getElementById('qVal').textContent=q+'%';

 var mat=D.material||'--';
 document.getElementById('matBadge').textContent=mat;
 if(D.mat_color)document.getElementById('matBadge').style.color='#'+D.mat_color;
 var mc=Math.round((D.mat_conf||0)*100);
 document.getElementById('matConf').style.width=mc+'%';
 document.getElementById('matConfVal').textContent=mc+'%';

 if(D.plan){
  document.getElementById('planForce').textContent=(D.plan.force||0).toFixed(1);
  document.getElementById('planSpeed').textContent=(D.plan.speed||0).toFixed(2);
  document.getElementById('planAper').textContent=(D.plan.aper||0).toFixed(0);
  document.getElementById('planRamp').textContent=(D.plan.ramp||0).toFixed(1);
 }

 drawImpedance();
 drawFFT();
 drawForce();
 drawCurvature();
 drawDepth();
}

function getCtx(id){var c=document.getElementById(id);c.width=c.clientWidth;c.height=c.clientHeight;return c.getContext('2d');}

function drawImpedance(){
 var ctx=getCtx('cImp'),w=ctx.canvas.width,h=ctx.canvas.height;
 ctx.fillStyle='#0d1220';ctx.fillRect(0,0,w,h);
 ctx.strokeStyle='#1e2a42';
 for(var i=0;i<6;i++){var x=10+i*(w-20)/5;ctx.beginPath();ctx.moveTo(x,5);ctx.lineTo(x,h-15);ctx.stroke();}
 for(var i=0;i<5;i++){var y=5+i*(h-20)/4;ctx.beginPath();ctx.moveTo(10,y);ctx.lineTo(w-10,y);ctx.stroke();}
 ctx.fillStyle='#5a6f8f';ctx.font='9px sans-serif';
 ctx.fillText('0',10,h-3);ctx.fillText('log|Z|',w/2-12,h-3);ctx.fillText('7',w-15,h-3);
 ctx.save();ctx.translate(8,h/2);ctx.rotate(-Math.PI/2);ctx.fillText('phase',-15,0);ctx.restore();

 // Plot material reference points
 var db=D.mat_db||[];
 for(var i=0;i<db.length;i++){
  var lz=db[i][0],ph=db[i][1],col=db[i][2];
  var px=10+(lz/7)*(w-20),py=5+((90+ph)/90)*(h-20);
  ctx.beginPath();ctx.arc(px,py,4,0,6.28);ctx.fillStyle='#'+col;ctx.globalAlpha=.4;ctx.fill();ctx.globalAlpha=1;
 }
 // Current measurement
 if(D.imp){
  var lz=D.imp.log_mag||0,ph=D.imp.phase||0;
  var px=10+(lz/7)*(w-20),py=5+((90+ph)/90)*(h-20);
  ctx.beginPath();ctx.arc(px,py,6,0,6.28);ctx.fillStyle='#4fc3f7';ctx.fill();
  ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
 }
}

function drawFFT(){
 var ctx=getCtx('cFFT'),w=ctx.canvas.width,h=ctx.canvas.height;
 ctx.fillStyle='#0d1220';ctx.fillRect(0,0,w,h);
 var sp=D.spectrum||[];
 if(sp.length===0)return;
 var mx=0;for(var i=0;i<sp.length;i++)if(sp[i]>mx)mx=sp[i];
 if(mx<1)mx=1;
 var bars=Math.min(sp.length,64);
 var bw=Math.max(1,(w-10)/bars);
 ctx.fillStyle='#9b59b6';
 for(var i=0;i<bars;i++){
  var bh=(sp[i]/mx)*(h-20);
  ctx.fillRect(5+i*bw,h-10-bh,bw-1,bh);
 }
 ctx.fillStyle='#5a6f8f';ctx.font='9px sans-serif';
 ctx.fillText('0 Hz',5,h-1);ctx.fillText('4 kHz',w-30,h-1);
 if(D.aco&&D.aco.dom_f){
  ctx.fillStyle='#f39c12';ctx.font='10px sans-serif';
  ctx.fillText('Peak: '+D.aco.dom_f.toFixed(0)+' Hz',w/2-30,12);
 }
}

function drawForce(){
 var ctx=getCtx('cForce'),w=ctx.canvas.width,h=ctx.canvas.height;
 ctx.fillStyle='#0d1220';ctx.fillRect(0,0,w,h);
 var f=D.forces||[0,0,0];
 var mx=0;for(var i=0;i<3;i++)if(f[i]>mx)mx=f[i];
 if(mx<0.5)mx=2;
 var bw=(w-40)/3,pad=10;
 var cols=['#e74c3c','#27ae60','#3498db'];
 var labels=['F1','F2','F3'];
 for(var i=0;i<3;i++){
  var bh=(f[i]/mx)*(h-30);
  var x=20+i*(bw+pad);
  ctx.fillStyle=cols[i];
  ctx.fillRect(x,h-15-bh,bw,bh);
  ctx.fillStyle='#e0e6f0';ctx.font='10px sans-serif';
  ctx.fillText(f[i].toFixed(1)+'N',x+2,h-18-bh);
  ctx.fillStyle='#5a6f8f';ctx.font='9px sans-serif';
  ctx.fillText(labels[i],x+bw/2-5,h-3);
 }
 if(D.slip){ctx.fillStyle='#e74c3c';ctx.font='bold 12px sans-serif';ctx.fillText('SLIP!',w/2-15,15);}
}

function drawCurvature(){
 var ctx=getCtx('cCurv'),w=ctx.canvas.width,h=ctx.canvas.height;
 ctx.fillStyle='#0d1220';ctx.fillRect(0,0,w,h);
 var cx=w/2,cy=h/2,r=Math.min(w,h)/2-15;
 // Cross-hair
 ctx.strokeStyle='#1e2a42';ctx.lineWidth=1;
 ctx.beginPath();ctx.moveTo(cx-r,cy);ctx.lineTo(cx+r,cy);ctx.stroke();
 ctx.beginPath();ctx.moveTo(cx,cy-r);ctx.lineTo(cx,cy+r);ctx.stroke();
 // Circle
 ctx.beginPath();ctx.arc(cx,cy,r,0,6.28);ctx.stroke();
 ctx.beginPath();ctx.arc(cx,cy,r*0.5,0,6.28);ctx.stroke();
 // Curvature dot
 var cvx=D.curv_x||0,cvy=D.curv_y||0;
 var dx=cx+cvx*r,dy=cy-cvy*r;
 ctx.beginPath();ctx.arc(dx,dy,8,0,6.28);ctx.fillStyle='#4fc3f7';ctx.fill();
 ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
 // Labels
 ctx.fillStyle='#5a6f8f';ctx.font='9px sans-serif';
 ctx.fillText('flat:'+((D.flatness||0)*100).toFixed(0)+'%',5,h-3);
 var gnames=['flat','convex','concave','edge','cyl'];
 ctx.fillText(gnames[D.geometry||0]||'?',w-35,h-3);
}

function drawDepth(){
 var g=document.getElementById('depthGrid');
 var d=D.depth||[];
 if(d.length!==64){
  if(g.childElementCount===0){for(var i=0;i<64;i++){var c=document.createElement('div');c.className='depth-cell';c.style.background='#1e2a42';g.appendChild(c);}}
  return;
 }
 if(g.childElementCount!==64){g.innerHTML='';for(var i=0;i<64;i++){var c=document.createElement('div');c.className='depth-cell';g.appendChild(c);}}
 var mn=9999,mx=0;
 for(var i=0;i<64;i++){if(d[i]>0&&d[i]<mn)mn=d[i];if(d[i]>mx)mx=d[i];}
 if(mx<=mn)mx=mn+1;
 var cells=g.children;
 for(var i=0;i<64;i++){
  var t=(d[i]-mn)/(mx-mn);
  var r2=Math.round(255*(1-t)),g2=Math.round(80+175*t),b2=Math.round(255*t);
  cells[i].style.background='rgb('+r2+','+g2+','+b2+')';
  cells[i].textContent=d[i]>0?d[i]:'';
 }
}
</script>
</body></html>
)rawliteral";


// ─────────────────────────────────────────────────────────────
// WebSocket event handler
// ─────────────────────────────────────────────────────────────
static void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WEB] Client #%u connected from %s\n",
                          client->id(), client->remoteIP().toString().c_str());
            break;

        case WS_EVT_DISCONNECT:
            Serial.printf("[WEB] Client #%u disconnected\n", client->id());
            break;

        case WS_EVT_DATA: {
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                // Parse JSON command
                StaticJsonDocument<128> doc;
                DeserializationError err = deserializeJson(doc, data, len);
                if (err) break;

                const char* cmd = doc["cmd"];
                if (cmd && _cmd_callback) {
                    _cmd_callback(cmd);
                }
            }
            break;
        }

        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}


void setupWiFiAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    delay(100); // let AP stabilize
    Serial.printf("[WEB] AP started: %s @ %s\n", WIFI_SSID,
                  WiFi.softAPIP().toString().c_str());
}


void setupWebServer() {
    _ws.onEvent(_onWsEvent);
    _server.addHandler(&_ws);

    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send_P(200, "text/html", DASHBOARD_HTML);
    });

    _server.begin();
    Serial.println("[WEB] Server started on port 80");
}


void setWebCommandCallback(WebCommandCallback cb) {
    _cmd_callback = cb;
}


void pushSensorData(const ImpedanceResult& imp,
                    const AcousticResult& aco,
                    const ForceResult& force,
                    const CurvatureResult& curv,
                    const DepthGrid& depth,
                    const GraspPlan& plan,
                    GraspState state,
                    float grasp_quality,
                    uint8_t fused_material,
                    float fused_confidence) {
    if (_ws.count() == 0) return;

    // Build JSON payload — sized for depth grid (64 ints) + spectrum (64 ints) + metadata
    StaticJsonDocument<4096> doc;

    doc["state"] = STATE_NAMES[state];
    doc["quality"] = grasp_quality;

    // Material
    if (fused_material < MATERIAL_COUNT) {
        doc["material"] = MATERIAL_DB[fused_material].name;
        char col_hex[7];
        snprintf(col_hex, sizeof(col_hex), "%06X",
                 (unsigned int)MATERIAL_DB[fused_material].led_color);
        doc["mat_color"] = col_hex;
    } else {
        doc["material"] = "Unknown";
        doc["mat_color"] = "5a6f8f";
    }
    doc["mat_conf"] = fused_confidence;

    // Impedance
    if (imp.valid) {
        JsonObject imp_obj = doc.createNestedObject("imp");
        imp_obj["log_mag"] = log10f(fmaxf(imp.magnitude, 0.01f));
        imp_obj["phase"] = imp.phase_deg;
        imp_obj["conf"] = imp.confidence;
    }

    // Material database reference points for the scatter plot
    JsonArray mat_db = doc.createNestedArray("mat_db");
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        JsonArray entry = mat_db.createNestedArray();
        entry.add(log10f(fmaxf(MATERIAL_DB[i].impedance_magnitude, 0.01f)));
        entry.add(MATERIAL_DB[i].impedance_phase_deg);
        char hex[7];
        snprintf(hex, sizeof(hex), "%06X", (unsigned int)MATERIAL_DB[i].led_color);
        entry.add(hex);
    }

    // Acoustic — send first 64 bins of spectrum
    if (aco.valid) {
        JsonObject aco_obj = doc.createNestedObject("aco");
        aco_obj["dom_f"] = aco.dominant_freq;
        aco_obj["centroid"] = aco.spectral_centroid;
        aco_obj["decay"] = aco.decay_ratio;

        JsonArray sp = doc.createNestedArray("spectrum");
        int bins = min(64, ACOUSTIC_FFT_SIZE / 2);
        for (int i = 0; i < bins; i++) {
            sp.add((int)aco.spectrum[i]);
        }
    }

    // Forces
    JsonArray forces_arr = doc.createNestedArray("forces");
    for (int i = 0; i < FSR_COUNT; i++) {
        forces_arr.add(force.forces_N[i]);
    }
    doc["slip"] = force.slip_detected;

    // Curvature
    doc["curv_x"] = curv.curvature_x;
    doc["curv_y"] = curv.curvature_y;
    doc["flatness"] = curv.flatness;
    doc["geometry"] = curv.geometry;

    // Depth grid (flat array of 64 values)
    if (depth.valid) {
        JsonArray depth_arr = doc.createNestedArray("depth");
        for (int r = 0; r < DEPTH_GRID_ROWS; r++) {
            for (int c = 0; c < DEPTH_GRID_COLS; c++) {
                depth_arr.add(depth.mm[r][c]);
            }
        }
    }

    // Grasp plan
    if (plan.valid) {
        JsonObject plan_obj = doc.createNestedObject("plan");
        plan_obj["force"] = plan.target_force_N;
        plan_obj["speed"] = plan.approach_speed;
        plan_obj["aper"] = plan.finger_aperture;
        plan_obj["ramp"] = plan.force_ramp_Nps;
    }

    // Serialize and broadcast
    size_t json_len = measureJson(doc);
    char* buf = (char*)malloc(json_len + 1);
    if (buf) {
        serializeJson(doc, buf, json_len + 1);
        _ws.textAll(buf, json_len);
        free(buf);
    }
}


void cleanupWebClients() {
    _ws.cleanupClients(WS_MAX_CLIENTS);
}
