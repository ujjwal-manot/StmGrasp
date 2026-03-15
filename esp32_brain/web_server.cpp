#include "web_server.h"
#include "hd_slip.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

static AsyncWebServer _server(WEB_PORT);
static AsyncWebSocket _ws("/ws");
static WebCommandCallback _cmd_callback = nullptr;

static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HYDRA Grasp v2</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0e17;color:#e0e6f0;font-family:'Segoe UI',system-ui,sans-serif;font-size:13px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px;padding:6px;max-width:1400px;margin:0 auto}
.panel{background:#141b2d;border:1px solid #1e2a42;border-radius:6px;padding:10px;min-height:160px}
.panel h3{font-size:11px;color:#5a6f8f;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px}
canvas{width:100%;height:120px;display:block;border-radius:4px}
.hdr{grid-column:1/-1;display:flex;justify-content:space-between;align-items:center;padding:6px 10px;background:#141b2d;border-radius:6px;border:1px solid #1e2a42}
.hdr h1{font-size:16px;font-weight:700;color:#4fc3f7}
.badge{display:inline-block;padding:3px 10px;border-radius:10px;font-weight:600;font-size:12px}
.ctrl{grid-column:1/-1;display:flex;gap:6px;justify-content:center}
.btn{padding:8px 20px;border:none;border-radius:5px;font-size:13px;font-weight:600;cursor:pointer;color:#fff}
.btn:hover{opacity:.85}
.btn-go{background:#27ae60}.btn-rel{background:#e67e22}.btn-stop{background:#e74c3c}.btn-cal{background:#2980b9}
.qbar{height:6px;background:#1e2a42;border-radius:3px;margin-top:4px;overflow:hidden}
.qfill{height:100%;border-radius:3px;transition:width .3s}
.stat-val{font-size:24px;font-weight:700;text-align:center;padding:6px 0}
.stat-label{font-size:10px;color:#5a6f8f;text-align:center}
.depth-grid{display:grid;grid-template-columns:repeat(8,1fr);gap:1px}
.depth-cell{aspect-ratio:1;border-radius:2px;font-size:6px;display:flex;align-items:center;justify-content:center}
.posterior-bar{display:flex;align-items:center;margin:2px 0;font-size:11px}
.posterior-bar .name{width:65px;text-align:right;padding-right:6px}
.posterior-bar .bar{flex:1;height:14px;background:#1e2a42;border-radius:3px;overflow:hidden;position:relative}
.posterior-bar .fill{height:100%;border-radius:3px;transition:width .3s}
.posterior-bar .pct{position:absolute;right:4px;top:0;line-height:14px;font-size:9px;color:#fff}
.log{font-family:monospace;font-size:10px;line-height:1.6;color:#8fa;max-height:140px;overflow-y:auto;padding:2px}
.wide{grid-column:span 2}
</style></head><body>
<div class="grid">
 <div class="hdr">
  <h1>HYDRA Grasp v2</h1>
  <span><span id="strategy" style="color:#f39c12;font-weight:700;margin-right:12px">--</span><span id="state" class="badge" style="background:#2980b9">IDLE</span></span>
 </div>

 <div class="panel"><h3>Impedance</h3><canvas id="cImp"></canvas></div>
 <div class="panel"><h3>Force</h3><canvas id="cForce"></canvas></div>
 <div class="panel"><h3>Depth (8x8)</h3><div id="depthGrid" class="depth-grid"></div></div>
 <div class="panel">
  <h3>Material Posterior (D-S)</h3>
  <div id="posterior"></div>
  <div style="margin-top:4px;font-size:10px;color:#5a6f8f">Conflict: <span id="conflict">0%</span> | Ignorance: <span id="ignorance">100%</span></div>
 </div>

 <div class="panel">
  <h3>Grasp Quality</h3>
  <div id="stateText" class="stat-val">IDLE</div>
  <div class="stat-label">Quality</div>
  <div class="qbar"><div id="qFill" class="qfill" style="width:0%;background:#27ae60"></div></div>
  <div id="qVal" class="stat-label">0%</div>
  <div class="stat-label" style="margin-top:6px">P(success)</div>
  <div class="qbar"><div id="spFill" class="qfill" style="width:0%;background:#9b59b6"></div></div>
  <div id="spVal" class="stat-label">0%</div>
 </div>
 <div class="panel">
  <h3>Plan</h3>
  <div style="font-size:11px;line-height:1.7;padding:2px 0">
   <div>Strategy: <b id="planStrat">--</b></div>
   <div>Force: <b id="planForce">--</b> N</div>
   <div>Speed: <b id="planSpeed">--</b></div>
   <div>Aperture: <b id="planAper">--</b> mm</div>
   <div>Ramp: <b id="planRamp">--</b> N/s</div>
  </div>
 </div>
 <div class="panel wide">
  <h3>Decision Log</h3>
  <div id="decLog" class="log">Waiting for data...</div>
 </div>

 <div class="ctrl">
  <button class="btn btn-go" onclick="ws_cmd('start')">START</button>
  <button class="btn btn-rel" onclick="ws_cmd('release')">RELEASE</button>
  <button class="btn btn-stop" onclick="ws_cmd('estop')">E-STOP</button>
  <button class="btn btn-cal" onclick="ws_cmd('calibrate')">CALIBRATE</button>
 </div>
</div>

<script>
var ws,D={};
function connect(){ws=new WebSocket('ws://'+location.host+'/ws');ws.onmessage=function(e){try{D=JSON.parse(e.data);render();}catch(x){}};ws.onclose=function(){setTimeout(connect,1000);};}
connect();
function ws_cmd(c){if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:c}));}

var matCols=['#3498DB','#F5B041','#9B59B6','#28B463','#EC7063','#D4AC0D'];
var matNames=['Metal','Skin','Plastic','Wood','Glass','Cardboard'];

function render(){
 var s=D.state||'IDLE';
 document.getElementById('state').textContent=s;
 document.getElementById('stateText').textContent=s;
 document.getElementById('strategy').textContent=D.strategy||'--';

 var q=Math.round((D.quality||0)*100);
 document.getElementById('qFill').style.width=q+'%';
 document.getElementById('qFill').style.background=q>60?'#27ae60':q>30?'#f39c12':'#e74c3c';
 document.getElementById('qVal').textContent=q+'%';

 var sp=Math.round((D.success_prob||0)*100);
 document.getElementById('spFill').style.width=sp+'%';
 document.getElementById('spFill').style.background=sp>70?'#9b59b6':sp>50?'#f39c12':'#e74c3c';
 document.getElementById('spVal').textContent=sp+'%';

 if(D.plan){
  document.getElementById('planStrat').textContent=D.plan.strat||'--';
  document.getElementById('planForce').textContent=(D.plan.force||0).toFixed(1);
  document.getElementById('planSpeed').textContent=(D.plan.speed||0).toFixed(2);
  document.getElementById('planAper').textContent=(D.plan.aper||0).toFixed(0);
  document.getElementById('planRamp').textContent=(D.plan.ramp||0).toFixed(1);
 }

 // DS Posterior
 var bel=D.belief||[0,0,0,0,0,0];
 var pl=D.plaus||[0,0,0,0,0,0];
 var ph=document.getElementById('posterior');
 var html='';
 for(var i=0;i<6;i++){
  var b=Math.round(bel[i]*100), p=Math.round(pl[i]*100);
  html+='<div class="posterior-bar"><span class="name">'+matNames[i]+'</span><div class="bar">';
  html+='<div class="fill" style="width:'+p+'%;background:'+matCols[i]+'33"></div>';
  html+='<div class="fill" style="width:'+b+'%;background:'+matCols[i]+';position:absolute;top:0;left:0;height:100%"></div>';
  html+='<span class="pct">'+b+'%</span></div></div>';
 }
 ph.innerHTML=html;
 document.getElementById('conflict').textContent=Math.round((D.conflict||0)*100)+'%';
 document.getElementById('ignorance').textContent=Math.round((D.ignorance||1)*100)+'%';

 // Decision log
 if(D.log&&D.log.length>0){
  document.getElementById('decLog').innerHTML=D.log.map(function(l){return'<div>'+l+'</div>';}).join('');
 }

 drawImpedance();drawForce();drawDepth();
}

function getCtx(id){var c=document.getElementById(id);c.width=c.clientWidth;c.height=c.clientHeight;return c.getContext('2d');}

function drawImpedance(){
 var ctx=getCtx('cImp'),w=ctx.canvas.width,h=ctx.canvas.height;
 ctx.fillStyle='#0d1220';ctx.fillRect(0,0,w,h);
 ctx.strokeStyle='#1e2a42';
 for(var i=0;i<6;i++){var x=10+i*(w-20)/5;ctx.beginPath();ctx.moveTo(x,5);ctx.lineTo(x,h-15);ctx.stroke();}
 var db=D.mat_db||[];
 for(var i=0;i<db.length;i++){
  var lz=db[i][0],ph2=db[i][1],col=db[i][2];
  var px=10+(lz/7)*(w-20),py=5+((90+ph2)/90)*(h-20);
  ctx.beginPath();ctx.arc(px,py,4,0,6.28);ctx.fillStyle='#'+col;ctx.globalAlpha=.4;ctx.fill();ctx.globalAlpha=1;
 }
 if(D.imp){
  var lz=D.imp.log_mag||0,ph2=D.imp.phase||0;
  var px=10+(lz/7)*(w-20),py=5+((90+ph2)/90)*(h-20);
  ctx.beginPath();ctx.arc(px,py,6,0,6.28);ctx.fillStyle='#4fc3f7';ctx.fill();
  ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
 }
 ctx.fillStyle='#5a6f8f';ctx.font='8px sans-serif';
 ctx.fillText('log|Z|',w/2-10,h-2);
}

function drawForce(){
 var ctx=getCtx('cForce'),w=ctx.canvas.width,h=ctx.canvas.height;
 ctx.fillStyle='#0d1220';ctx.fillRect(0,0,w,h);
 var f=D.forces||[0,0,0];
 var mx=0;for(var i=0;i<3;i++)if(f[i]>mx)mx=f[i];
 if(mx<0.5)mx=2;
 var bw=(w-40)/3,pad=10;
 var cols=['#e74c3c','#27ae60','#3498db'];
 for(var i=0;i<3;i++){
  var bh=(f[i]/mx)*(h-25);var x=20+i*(bw+pad);
  ctx.fillStyle=cols[i];ctx.fillRect(x,h-12-bh,bw,bh);
  ctx.fillStyle='#e0e6f0';ctx.font='9px sans-serif';ctx.fillText(f[i].toFixed(1)+'N',x+2,h-15-bh);
 }
 if(D.slip){ctx.fillStyle='#e74c3c';ctx.font='bold 11px sans-serif';ctx.fillText('SLIP!',w/2-12,14);}
 if(D.hd_ready){ctx.fillStyle='#9b59b6';ctx.font='9px sans-serif';ctx.fillText('HD',w-20,14);}
}

function drawDepth(){
 var g=document.getElementById('depthGrid');var d=D.depth||[];
 if(d.length!==64){if(g.childElementCount===0)for(var i=0;i<64;i++){var c=document.createElement('div');c.className='depth-cell';c.style.background='#1e2a42';g.appendChild(c);}return;}
 if(g.childElementCount!==64){g.innerHTML='';for(var i=0;i<64;i++){var c=document.createElement('div');c.className='depth-cell';g.appendChild(c);}}
 var mn=9999,mx2=0;for(var i=0;i<64;i++){if(d[i]>0&&d[i]<mn)mn=d[i];if(d[i]>mx2)mx2=d[i];}
 if(mx2<=mn)mx2=mn+1;var cells=g.children;
 for(var i=0;i<64;i++){var t=(d[i]-mn)/(mx2-mn);var r=Math.round(255*(1-t)),g2=Math.round(80+175*t),b=Math.round(255*t);cells[i].style.background='rgb('+r+','+g2+','+b+')';cells[i].textContent=d[i]>0?d[i]:'';}
}
</script></body></html>
)rawliteral";

static void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, data, len)) return;
            const char* cmd = doc["cmd"];
            if (cmd && _cmd_callback) _cmd_callback(cmd);
        }
    }
}

void setupWiFiAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    delay(100);
    Serial.printf("[WEB] AP: %s @ %s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());
}

void setupWebServer() {
    _ws.onEvent(_onWsEvent);
    _server.addHandler(&_ws);
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){ req->send_P(200, "text/html", DASHBOARD_HTML); });
    _server.begin();
}

void setWebCommandCallback(WebCommandCallback cb) { _cmd_callback = cb; }

void pushSensorData(const ImpedanceResult& imp, const AcousticResult& aco,
                    const ForceResult& force, const CurvatureResult& curv,
                    const DepthGrid& depth, const GraspPlan& plan,
                    GraspState state, float grasp_quality, const DSResult& ds) {
    if (_ws.count() == 0) return;

    StaticJsonDocument<4096> doc;
    doc["state"] = STATE_NAMES[state];
    doc["quality"] = grasp_quality;
    doc["success_prob"] = plan.success_prob;
    doc["slip"] = force.slip_detected;
    doc["hd_ready"] = hdIsReady();

    // DS fusion results
    if (ds.valid) {
        doc["strategy"] = STRATEGY_NAMES[plan.strategy];
        JsonArray bel = doc.createNestedArray("belief");
        JsonArray pl = doc.createNestedArray("plaus");
        for (int i = 0; i < MATERIAL_COUNT; i++) {
            bel.add(ds.belief[i]);
            pl.add(ds.plausibility[i]);
        }
        doc["conflict"] = ds.conflict;
        doc["ignorance"] = ds.mass_theta;
    }

    // Decision log
    const DSDecisionLog* log = dsGetLog();
    if (log->count > 0) {
        JsonArray la = doc.createNestedArray("log");
        for (int i = 0; i < log->count; i++) la.add(log->messages[i]);
    }

    // Impedance
    if (imp.valid) {
        JsonObject io = doc.createNestedObject("imp");
        io["log_mag"] = log10f(fmaxf(imp.magnitude, 0.01f));
        io["phase"] = imp.phase_deg;
    }

    // Material DB reference
    JsonArray mdb = doc.createNestedArray("mat_db");
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        JsonArray e = mdb.createNestedArray();
        e.add(log10f(fmaxf(MATERIAL_DB[i].impedance_magnitude, 0.01f)));
        e.add(MATERIAL_DB[i].impedance_phase_deg);
        char hex[7]; snprintf(hex, 7, "%06X", (unsigned)MATERIAL_DB[i].led_color);
        e.add(hex);
    }

    // Forces
    JsonArray fa = doc.createNestedArray("forces");
    for (int i = 0; i < FSR_COUNT; i++) fa.add(force.forces_N[i]);

    // Depth
    if (depth.valid) {
        JsonArray da = doc.createNestedArray("depth");
        for (int r = 0; r < DEPTH_GRID_ROWS; r++)
            for (int c = 0; c < DEPTH_GRID_COLS; c++)
                da.add(depth.mm[r][c]);
    }

    // Plan
    if (plan.valid) {
        JsonObject po = doc.createNestedObject("plan");
        po["strat"] = STRATEGY_NAMES[plan.strategy];
        po["force"] = plan.target_force_N;
        po["speed"] = plan.approach_speed;
        po["aper"] = plan.finger_aperture;
        po["ramp"] = plan.force_ramp_Nps;
    }

    size_t jl = measureJson(doc);
    char* buf = (char*)malloc(jl + 1);
    if (buf) {
        serializeJson(doc, buf, jl + 1);
        _ws.textAll(buf, jl);
        free(buf);
    }
}

void cleanupWebClients() { _ws.cleanupClients(WS_MAX_CLIENTS); }
