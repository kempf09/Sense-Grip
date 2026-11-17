// Embedded single-page dashboard (served by GET /). Inline CSS + vanilla JS only:
// the ESP32 AP has no internet, so no external URLs.
#pragma once
#include <Arduino.h>

const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SenseGrip</title>
<style>
:root{--bg:#10151c;--pn:#1a222d;--bd:#2c3846;--tx:#e6edf3;--mu:#8b98a8;--ok:#2ea043;--bad:#e5484d;--wn:#e0a030;--ac:#3b82f6}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font:15px system-ui,Arial,sans-serif}
header{display:flex;flex-wrap:wrap;align-items:center;gap:12px;padding:8px 12px;background:var(--pn);border-bottom:1px solid var(--bd)}
header h1{font-size:17px;margin:0;flex:1}
nav{display:flex;gap:4px;padding:8px 12px 0;flex-wrap:wrap}
nav button{background:var(--pn);color:var(--mu);border:1px solid var(--bd);border-bottom:0;border-radius:6px 6px 0 0;padding:8px 14px;font-size:15px;cursor:pointer}
nav button.on{color:var(--tx);background:var(--bd)}
main{padding:12px;max-width:1100px;margin:0 auto}
.tab{display:none}.tab.on{display:block}
.panel{background:var(--pn);border:1px solid var(--bd);border-radius:8px;padding:12px;margin-bottom:12px}
.panel h2{font-size:14px;margin:0 0 8px;color:var(--mu);text-transform:uppercase;letter-spacing:.05em}
.grid{display:grid;gap:12px;grid-template-columns:1fr 1fr}
.grid3{display:grid;gap:12px;grid-template-columns:repeat(3,1fr)}
@media(max-width:700px){.grid,.grid3{grid-template-columns:1fr}}
#banner{display:none;background:var(--bad);color:#fff;text-align:center;padding:8px;font-weight:700}
#toast{position:fixed;left:50%;bottom:16px;transform:translateX(-50%);background:var(--bad);color:#fff;padding:10px 16px;border-radius:8px;display:none;max-width:90%;z-index:9}
#badge{font-size:34px;font-weight:800;text-align:center;padding:14px 8px;border-radius:10px;background:#444;letter-spacing:.04em}
.st-IDLE{background:#4b5563!important}.st-START{background:#2563eb!important}.st-PRE_TOUCH{background:#0891b2!important}
.st-GRASP{background:#d97706!important}.st-MONITOR{background:#16a34a!important}.st-OPEN{background:#7c3aed!important}.st-FAULT{background:var(--bad)!important}
#faultBox{display:none;margin-top:8px;padding:8px;border:2px solid var(--bad);border-radius:8px;color:#ffb4b6;font-weight:600}
.small{color:var(--mu);font-size:13px}
.pill{display:inline-block;padding:3px 10px;border-radius:999px;font-weight:700;font-size:13px;background:#444}
.pill.on{background:var(--ok)}.pill.off{background:#555}
#blk div{padding:5px 8px;margin:3px 0;border-radius:5px;background:rgba(229,72,77,.15);color:#ffb4b6;border-left:4px solid var(--bad)}
#blk div.ready{background:rgba(46,160,67,.18);color:#8ee6a0;border-left-color:var(--ok);font-weight:700}
#blk div.clear{background:rgba(46,160,67,.18);color:#8ee6a0;border-left-color:var(--ok)}
#last{padding:8px;border-radius:6px;background:#222c38}.ok{color:#8ee6a0}.fail{color:#ffb4b6;font-weight:600}
.bar{position:relative;height:22px;background:#0c1117;border:1px solid var(--bd);border-radius:5px;overflow:hidden}
.bar>i{position:absolute;left:0;top:0;bottom:0;background:var(--ac)}
.bar>span{position:absolute;left:8px;top:2px;font-size:12px;text-shadow:0 0 3px #000}
.frow{display:flex;align-items:center;gap:8px;margin:6px 0}.frow b{width:62px}.frow .bar{flex:1}
.dot{width:14px;height:14px;border-radius:50%;background:#444;flex:none}.dot.on{background:#22d3ee;box-shadow:0 0 6px #22d3ee}
button.b,.row button{background:var(--bd);color:var(--tx);border:1px solid #3d4b5c;border-radius:8px;padding:10px 14px;font-size:15px;cursor:pointer;touch-action:manipulation}
button.b:disabled{opacity:.45;cursor:not-allowed}
button.pri{background:var(--ac);border-color:var(--ac)}
.estop{width:100%;padding:22px;font-size:26px;font-weight:900;background:#c4161c;border:3px solid #ff6b6f;color:#fff;border-radius:12px;margin-top:8px;cursor:pointer}
.btns{display:flex;gap:8px;flex-wrap:wrap}.btns .b{flex:1;min-width:90px}
canvas{width:100%;aspect-ratio:1;background:#0c1117;border:1px solid var(--bd);border-radius:6px;display:block}
.hm{text-align:center}.hm .small{margin-top:4px}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{padding:4px 6px;border-bottom:1px solid var(--bd);text-align:left}
input[type=number],input[type=text],select{background:#0c1117;color:var(--tx);border:1px solid var(--bd);border-radius:5px;padding:6px;width:100%;font-size:14px}
label.f{display:block;font-size:13px;color:var(--mu)}
select{width:auto;min-width:150px;display:block}
.cfg{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px;margin:8px 0}
.jog{user-select:none;-webkit-user-select:none;touch-action:none}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:6px 0}
.fcal input{width:64px;padding:4px}
</style></head><body>
<div id="banner">Connection lost - retrying...</div>
<header><h1>SenseGrip</h1><span id="hdrFw" class="small" style="margin-left:auto;font-size:11px"></span><span id="hdrState" class="pill">...</span></header>
<nav id="tabs"><button data-t="comb" class="on">Combined</button><button data-t="servo">Servo</button><button data-t="fsr">FSR</button><button data-t="tof">SEN0628</button></nav>
<main>
<section class="tab on" id="t-comb">
 <div class="grid">
  <div>
   <div class="panel"><div id="badge">...</div>
    <div id="faultBox"></div>
    <div class="row"><span class="small">Armed:</span><span id="armed" class="pill off">DISARMED</span><span class="small" id="reason"></span></div>
    <div class="row"><span class="small">Position:</span><span id="homed" class="pill off">NOT HOMED</span><span class="small homeMsg"></span></div></div>
   <div class="panel"><h2>Blockers</h2><div id="blk"></div></div>
   <div class="panel"><h2>Last action</h2><div id="last" class="small">-</div></div>
   <div class="panel"><h2>Recordings</h2>
    <div class="row"><button class="b" id="recMan">Start manual recording</button><span class="small" id="recStat">collector: starting...</span></div>
    <div class="row" style="margin-top:6px"><span class="small">Mark:</span><button class="b" data-mark="pull">Pull (P)</button><button class="b" data-mark="twist">Twist (T)</button><button class="b" data-mark="slide">Slide (S)</button><button class="b" data-mark="other">Other (O)</button><button class="b" data-mark="end">End (E)</button></div>
    <div id="recList" class="small" style="margin-top:6px">-</div>
    <div class="btns" style="margin-top:6px"><button class="b" id="recAll">Download all</button><button class="b" id="recClr">Clear all</button></div>
    <p class="small">Every grasp (1 s before START until 1 s after it is back at IDLE, or after FAULT) is recorded at 100 Hz while this page is open and kept in this browser, with the raw SEN0628 frames in a second file (_tof.csv). Mark keys P/T/S/O/E work anywhere on the page (not while typing in a field). CSV: forces in N (f = filtered, r = raw, FSR1..6), position in servo counts, sizes in mm.</p></div>
   <div class="panel"><div class="btns">
     <button class="b" id="bHome">Home</button><button class="b pri" id="bArm">Arm</button><button class="b" id="bOpen">Open</button><button class="b" id="bReset">Reset</button></div>
     <button class="estop" id="estop">E-STOP</button></div>
  </div>
  <div>
   <div class="panel"><h2>Finger force</h2><div id="fingers"></div>
    <h2 style="margin-top:12px">Position</h2><div class="bar"><i id="posBar"></i><span id="posTxt"></span></div></div>
   <div class="panel"><h2>Object</h2><div id="cObj" class="small">-</div></div>
   <div class="panel"><h2>ToF (SEN0628)</h2><div class="grid3" id="cHm"></div></div>
  </div>
 </div>
</section>

<section class="tab" id="t-servo">
 <div class="grid">
  <div class="panel"><h2>Telemetry</h2><table id="svTbl"></table></div>
  <div class="panel"><h2>Manual</h2>
   <div class="row"><label class="f">Jog speed (1-61, x0.732 RPM)<input type="number" id="jogSpd" min="1" max="61" value="20"></label><label class="f"><span>Ignore limits while jogging</span><input type="checkbox" id="jogIgn"></label></div>
   <div class="btns"><button class="b jog" id="jogC">Jog Close (hold)</button><button class="b jog" id="jogO">Jog Open (hold)</button></div>
   <div class="btns" style="margin-top:8px"><button class="b" id="setOpen">Set OPEN zero here</button><button class="b" id="setClose">Set close limit here</button></div>
   <div class="btns" style="margin-top:8px"><button class="b" id="clearLim">Clear limits</button><button class="b" id="posMode">Set position mode (0)</button><button class="b" id="calMid">Calibrate servo middle here</button><button class="b" id="scan">Scan</button></div>
   <p class="small">Jog is only accepted while IDLE and disarmed. After jogging or setting limits, press Open again before arming. Limits: <span id="limTxt">-</span></p>
   <button class="estop" id="estop2">E-STOP</button></div>
  <div class="panel"><h2>Position reference</h2>
   <div class="row"><span id="homed2" class="pill off">NOT HOMED</span><span class="small homeMsg"></span></div>
   <p class="small" id="estTxt">SEN0628 estimate: -</p>
   <div class="btns"><button class="b pri" id="bHome2">Home (close to touch)</button><button class="b" id="bSweep">Run calibration sweep</button></div>
   <p class="small" id="swTxt">Calibration sweep: -</p>
   <p class="small">Home: gripper empty; the fingers close slowly until a fingertip pad touches, then open. Sweep (first-time setup, or after mechanical changes): gripper empty and nothing in the SEN0628 view; records the sensors from OPEN zero to touch.</p>
   <canvas id="swChart" style="aspect-ratio:auto;height:220px;touch-action:none;cursor:crosshair"></canvas>
   <p class="small">Fingertip opening (mm from the gripper axis) vs motor position, from the stored sweep. Drag a point up/down to correct it (orange = edited; grey = analysed value). Edits are saved with the sweep and cleared by a new sweep.</p>
   <div class="row"><span class="small" id="swSel">No point selected</span>
    <input type="number" step="0.1" min="0" max="250" id="swMm" style="width:80px"><button class="b" id="swSet">Set</button>
    <button class="b" id="swUndo">Undo point</button><button class="b" id="swClr">Clear all edits</button></div>
   <h2 style="margin-top:12px">Sweep settings</h2><div class="cfg" id="swCfg"></div><button class="b pri" id="swSave">Save sweep settings</button></div>
 </div>
 <div class="panel"><h2>Config</h2><div class="cfg" id="svCfg"></div><button class="b pri" id="svSave">Save</button></div>
</section>

<section class="tab" id="t-fsr">
 <div class="panel"><h2>Channels</h2><div style="overflow-x:auto"><table id="fsrTbl"></table></div></div>
 <div class="panel"><h2>Filter</h2>
  <div class="row">
   <label class="f">Filter<select id="flKind"><option value="0">None (raw)</option><option value="1">Moving average</option><option value="2">Median</option><option value="3">Exponential (EMA)</option><option value="4">Kalman (1-D)</option></select></label>
   <label class="f" id="flWinL">Window N<input type="number" id="flWin" min="2" max="20"></label>
   <label class="f" id="flAlphaL">&alpha; (0.05-1)<input type="number" id="flAlpha" min="0.05" max="1" step="0.05"></label>
   <label class="f" id="flQL">Q<input type="number" id="flQ" min="0.00001" max="10" step="0.001"></label>
   <label class="f" id="flRL">R<input type="number" id="flR" min="0.00001" max="10" step="0.001"></label>
  </div>
  <div class="btns" style="margin-top:8px"><button class="b pri" id="flSave">Apply filter</button><span class="small" id="flDelay"></span></div>
  <p class="small" id="flLimit"></p>
  <div class="row" style="margin-top:10px"><label class="f">Chart channel<select id="flCh"></select></label>
   <span class="small"><span style="color:#8a96a3">&#9632;</span> raw &nbsp; <span style="color:#4ea1ff">&#9632;</span> filtered &nbsp; force in N, last 10 s</span></div>
  <canvas id="flChart" style="aspect-ratio:auto;height:220px"></canvas>
 </div>
 <div class="panel"><h2>Calibration (per channel)</h2><div id="calBox"></div>
  <p class="small">r = fixed resistor (ohm); f1/o1, f2/o2 = two force (N) / resistance (ohm) points; z = zero mV. Only in IDLE or FAULT.</p></div>
 <div class="panel"><h2>Config</h2><div class="cfg" id="fsrCfg"></div><button class="b pri" id="fsrSave">Save</button></div>
</section>

<section class="tab" id="t-tof">
 <div class="panel"><div class="grid3" id="tHm"></div>
  <p class="small">Click a cell to toggle it in the zone mask (hatched = ignored). Then Save mask.</p>
  <button class="b pri" id="maskSave">Save mask</button></div>
 <div class="panel"><h2>Health</h2><table id="tofTbl"></table></div>
 <div class="panel"><h2>Config</h2><div class="cfg" id="tofCfg"></div><button class="b pri" id="tofSave">Save</button></div>
 <div class="panel"><h2>Object classification</h2>
  <div id="objLive" class="small">-</div>
  <div class="row" style="margin-top:8px"><label class="small">Class override <select id="objOv"><option value="auto">auto</option><option value="small">small</option><option value="medium">medium</option><option value="large">large</option></select></label>
   <label class="small"><input type="checkbox" id="objDel"> Delicate</label>
   <label class="small"><input type="checkbox" id="objAppr"> Approach only</label>
   <span class="small">(not kept after restart)</span></div>
  <div class="cfg" id="objCfg" style="margin-top:8px"></div>
  <div class="btns"><button class="b pri" id="objSave">Save class settings</button><button class="b" id="objBg">Capture background</button></div>
  <p class="small">Stand-off = x mm + 10 % of the object distance. Cyan outlines on the maps = object zones; magenta = finger zones.</p></div>
</section>
</main>
<div id="toast"></div>
<script>
'use strict';
var $=function(i){return document.getElementById(i)};
var FSR_PRINT=[2,1,3,4,5,6], FMAP=[[5,0],[3,2],[4,1]], FNAME=['Finger 1','Finger 2','Finger 3'];
var S=null, T=null, CFG=null, tab='comb', fails=0, stPend=false, tfPend=false;

function esc(s){return String(s).replace(/[&<>"]/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function fx(v,d){return (v===null||v===undefined||isNaN(v))?'-':Number(v).toFixed(d)}
var toastT=0;
function toast(m){var t=$('toast');t.textContent=m;t.style.display='block';clearTimeout(toastT);toastT=setTimeout(function(){t.style.display='none'},4000)}
function post(url,obj){
  return fetch(url,{method:'POST',body:new URLSearchParams(obj||{})}).then(function(r){
    return r.json().catch(function(){return {ok:false,error:'HTTP '+r.status}}).then(function(j){
      if(!r.ok||j.ok===false){toast(j.error||('HTTP '+r.status));return null}
      return j;
    });
  }).catch(function(){toast('Request failed (no connection)');return null});
}
function getJ(url){return fetch(url,{cache:'no-store'}).then(function(r){if(!r.ok)throw new Error(r.status);return r.json()})}

/* ---------- tabs ---------- */
document.querySelectorAll('#tabs button').forEach(function(b){b.onclick=function(){
  tab=b.dataset.t;
  document.querySelectorAll('#tabs button').forEach(function(x){x.classList.toggle('on',x===b)});
  document.querySelectorAll('.tab').forEach(function(x){x.classList.toggle('on',x.id==='t-'+tab)});
  if(tab==='servo'||tab==='fsr'||tab==='tof')loadCfg();
  if(tab==='fsr')loadCal();
  if(tab==='comb'||tab==='tof')pollTof();
  render();
}});

/* ---------- heatmaps ---------- */
function colorFor(mm){var c=Math.max(0,Math.min(400,mm));return 'hsl('+Math.round(c/400*240)+',85%,'+(c<200?50:45)+'%)'}
function makeHm(host,id,title,clickable){
  var d=document.createElement('div');d.className='hm';
  d.innerHTML='<canvas width="256" height="256" id="'+id+'"></canvas><div class="small" id="'+id+'t">'+title+'</div>';
  host.appendChild(d);
  var cv=d.firstChild;
  if(clickable)cv.onclick=function(e){
    var r=cv.getBoundingClientRect();
    var c=Math.floor((e.clientX-r.left)/r.width*8),rw=Math.floor((e.clientY-r.top)/r.height*8);
    if(c<0||c>7||rw<0||rw>7)return;
    if(!CFG)return;var k=+id.slice(-1);maskEdit[k]^=(1n<<BigInt(rw*8+c));drawTof();
  };
}
var FZ=[0n,0n,0n],OZ=[0n,0n,0n],SW=null;
function drawHm(cv,sn,mask,nums,idx){
  var g=cv.getContext('2d'),W=cv.width,w=W/8;
  g.clearRect(0,0,W,W);g.font='bold 11px sans-serif';g.textAlign='center';g.textBaseline='middle';
  var off=!sn||!sn.online||!sn.mm;
  for(var z=0;z<64;z++){
    var x=(z%8)*w,y=Math.floor(z/8)*w;
    if(off){g.fillStyle='#2a323d'}else{g.fillStyle=colorFor(sn.mm[z])}
    g.fillRect(x,y,w-1,w-1);
    var masked=mask!==null&&!((mask>>BigInt(z))&1n);
    if(masked&&!off){g.fillStyle='rgba(0,0,0,.6)';g.fillRect(x,y,w-1,w-1)}
    if(masked){g.strokeStyle='rgba(255,255,255,.55)';g.lineWidth=1;g.beginPath();
      for(var k=-w;k<w;k+=6){g.moveTo(x+k,y+w);g.lineTo(x+k+w,y)}
      g.save();g.beginPath();g.rect(x,y,w-1,w-1);g.clip();g.stroke();g.restore()}
    if(nums&&!off){g.fillStyle='#000';g.fillText(sn.mm[z],x+w/2,y+w/2)}
    if(idx!==undefined&&((FZ[idx]>>BigInt(z))&1n)){g.strokeStyle='#ff4fd8';g.lineWidth=2;g.strokeRect(x+1.5,y+1.5,w-4,w-4)}
    if(idx!==undefined&&((OZ[idx]>>BigInt(z))&1n)){g.strokeStyle='#39d0ff';g.lineWidth=3;g.strokeRect(x+3,y+3,w-7,w-7)}
  }
  if(off){g.fillStyle='#fff';g.font='bold 26px sans-serif';g.fillText('offline',W/2,W/2)}
}

/* ---------- Combined ---------- */
function mkFingers(){
  var h='';for(var i=0;i<3;i++)h+='<div class="frow"><b>'+FNAME[i]+'</b><div class="bar"><i id="fb'+i+'"></i><span id="ft'+i+'"></span></div><span class="dot" id="fd'+i+'"></span></div>';
  $('fingers').innerHTML=h;
  for(var k=0;k<3;k++){makeHm($('cHm'),'ch'+k,'S'+(k+1),false);makeHm($('tHm'),'th'+k,'S'+(k+1),true)}
}
function render(){
  if(!S)return;
  flPush();
  var st=S.state||'?';
  var b=$('badge');b.textContent=st;b.className='st-'+st;
  var h=$('hdrState');h.textContent=st;h.className='pill st-'+st;
  var dg=S.diag||{};$('hdrFw').textContent=(dg.fw||'old firmware?')+' | up '+(dg.uptimeS||0)+' s | last reset: '+(dg.resetReason||'?')+' | stack free ctl '+(dg.controlStackFree||'?')+' / web '+(dg.webStackFree||'?');
  var fb=$('faultBox');
  if(st==='FAULT'){fb.style.display='block';fb.innerHTML='FAULT while in '+esc(S.faultFrom)+': '+esc(S.faultReason||S.reason)}else fb.style.display='none';
  var a=$('armed');a.textContent=S.armed?'ARMED':'DISARMED';a.className='pill '+(S.armed?'on':'off');
  renderHome();
  renderObject();
  $('reason').textContent=S.reason||'';
  var bl=S.blockers||[], only=(bl.length===1&&bl[0]==='not armed');
  var hb='';
  if(bl.length===0)hb='<div class="clear">No blockers</div>';
  else if(only)hb='<div class="ready">Ready to arm</div>';
  else bl.forEach(function(t){hb+='<div>'+esc(t)+'</div>'});
  $('blk').innerHTML=hb;
  var la=S.lastAction||{};var l=$('last');l.textContent=la.msg||'-';l.className=la.msg?(la.ok?'ok':'fail'):'small';
  var armB=$('bArm');
  armB.textContent=S.armed?'Disarm':'Arm';
  var armBlock=S.armed?[]:bl.filter(function(t){return t!=='not armed'});
  armB.disabled=armBlock.length>0;armB.title=armBlock.length?armBlock[0]:'';
  $('bOpen').disabled=false;
  var f=S.fsr||{};
  for(var i=0;i<3;i++){
    var v=(f.fingerForce||[])[i]||0,mx=(CFG&&CFG.maxForceN)||10;
    $('fb'+i).style.width=Math.max(0,Math.min(100,v/mx*100))+'%';
    $('fb'+i).style.background=(f.fingerContact||[])[i]?'var(--ok)':'var(--ac)';
    $('ft'+i).textContent=fx(v,2)+' N';
    $('fd'+i).className='dot'+((f.fingerContact||[])[i]?' on':'');
  }
  var L=S.limits||{},pb=$('posBar'),pt=$('posTxt'),pos=(S.servo||{}).position;
  if(!L.valid){pb.style.width='0';pt.textContent='limits not set'}
  else{var sp=L.closeLimit-L.openZero,fr=sp?(pos-L.openZero)/sp:0;
    pb.style.width=Math.max(0,Math.min(100,fr*100))+'%';
    pt.textContent=pos+'  ['+L.openZero+' .. '+L.closeLimit+']'}
  if(tab==='servo'){renderServo();drawSweep();}
  if(tab==='fsr')renderFsr();
  drawTof();
}
$('bArm').onclick=function(){post(S&&S.armed?'/api/fsm/disarm':'/api/fsm/arm')};
$('bOpen').onclick=function(){post('/api/fsm/open')};
$('bReset').onclick=function(){post('/api/fsm/reset')};
$('estop').onclick=$('estop2').onclick=function(){post('/api/fsm/estop')};   /* never disabled */

/* ---------- ToF ---------- */
var maskEdit=[0n,0n,0n],maskLoaded=[null,null,null];
function hex16(m){return BigInt.asUintN(64,m).toString(16).padStart(16,'0')}
function drawTof(){
  var mins=(S&&S.tof&&S.tof.minMm)||[];
  for(var k=0;k<3;k++){
    var sn=T&&T.sensors?T.sensors[k]:null;
    if(tab==='comb'){
      drawHm($('ch'+k),sn,null,false,k);
      var mm=sn&&sn.online?sn.minMm:(mins[k]===undefined?null:mins[k]);
      $('ch'+k+'t').textContent='S'+(k+1)+(mm===null?' offline':'  min '+mm+' mm');
    }
    if(tab==='tof'){
      drawHm($('th'+k),sn,CFG?maskEdit[k]:null,true,k);
      $('th'+k+'t').textContent='S'+(k+1)+(sn&&sn.online?'  min '+sn.minMm+' mm':' offline');
    }
  }
  if(tab==='tof'&&T&&T.sensors){
    var STG=['-','no I2C ACK','begin()','setRangingMode()'];
    var h='<tr><th>Sensor</th><th>Online</th><th>Init</th><th>ok</th><th>err</th><th>last rc</th><th>age ms</th><th>max read ms</th><th>dropouts (moving)</th><th>init fails (last stage)</th></tr>';
    T.sensors.forEach(function(s,k){h+='<tr><td>S'+(k+1)+'</td><td>'+(s.online?'yes':'no')+'</td><td>'+(s.initialized?'yes':'NO')+'</td><td>'+s.ok+'</td><td>'+s.err+'</td><td>'+s.lastRc+'</td><td>'+(s.ageMs===null?'-':s.ageMs)+'</td><td>'+fx(s.readUsMax/1000,1)+'</td><td>'+s.dropouts+' ('+s.dropoutsMoving+')</td><td>'+s.initFails+' ('+(STG[s.initFailStage]||s.initFailStage)+')</td></tr>'});
    var d=S.diag||{};
    h+='<tr><td colspan="10" class="small">Control loop max '+fx((d.loopMaxUs||0)/1000,1)+' ms (5 s) &middot; FSR pass max '+fx((d.fsrSampleUsMax||0)/1000,1)+' ms &middot; servo bus errors '+(d.servoBusErrors||0)+' &middot; position jumps '+(d.posJumps||0)+'</td></tr>';
    $('tofTbl').innerHTML=h;
  }
}

/* ---------- config forms ---------- */
var FORMS={
 svCfg:[['approachSpeed','Approach speed (1-61, x0.732 RPM)'],['graspSpeed','Grasp speed (1-61, x0.732 RPM)'],['openSpeed','Open speed (1-61, x0.732 RPM)'],['closeTorqueLimit','Close max current (x6.5 mA)'],['openTorqueLimit','Open max current (x6.5 mA)'],['preTouchTimeoutMs','Pre-touch timeout ms'],['graspTimeoutMs','Grasp timeout ms'],['openTimeoutMs','Open timeout ms'],['openToleranceCounts','Open tolerance counts'],['limitSlackCounts','Limit slack counts'],['closePositive','Close = positive','b']],
 fsrCfg:[['contactForceN','Contact force N'],['contactSamples','Contact samples'],['graspSamples','Grasp samples'],['maxForceN','Max force N'],['minActiveFsr','Min FSR channels in contact']],
 tofCfg:[['detectMm','Detect distance mm'],['detectHoldMs','Detect hold ms']],
 swCfg:[['sweepStepCounts','Step counts per sample (60-500)'],['sweepSpeed','Sweep speed (1-20)'],['sweepCurrent','Sweep max current (x6.5 mA, 0 = Close max current)']],
 objCfg:[['smallMaxMm','Small below width mm'],['mediumMaxMm','Medium below width mm'],['contactSpeed','Slow-to-contact speed (1-61)'],
  ['smallStandoffMm','Small: stand-off mm'],['smallForceN','Small: target force N'],['smallMaxCurrent','Small: max current (x6.5 mA)'],['smallGraspSpeed','Small: grasp speed'],
  ['mediumStandoffMm','Medium: stand-off mm'],['mediumForceN','Medium: target force N'],['mediumMaxCurrent','Medium: max current (x6.5 mA)'],['mediumGraspSpeed','Medium: grasp speed'],
  ['largeStandoffMm','Large: stand-off mm'],['largeForceN','Large: target force N'],['largeMaxCurrent','Large: max current (x6.5 mA)'],['largeGraspSpeed','Large: grasp speed'],
  ['delicateStandoffMm','Delicate: stand-off mm'],['delicateForceN','Delicate: target force N'],['delicateMaxCurrent','Delicate: max current (x6.5 mA)'],['delicateGraspSpeed','Delicate: grasp speed']]
};
function buildForms(){
  Object.keys(FORMS).forEach(function(id){
    var h='';FORMS[id].forEach(function(f){
      if(f[2]==='b')h+='<label class="f"><input type="checkbox" data-k="'+f[0]+'"> '+f[1]+'</label>';
      else h+='<label class="f">'+f[1]+'<input type="number" step="any" data-k="'+f[0]+'"></label>';
    });
    $(id).innerHTML=h;
  });
}
function fillForms(){
  document.querySelectorAll('.cfg [data-k]').forEach(function(el){
    var v=CFG[el.dataset.k];
    if(el.type==='checkbox'){el.checked=!!v;el.dataset.o=v?'1':'0'}
    else{el.value=v;el.dataset.o=String(v)}
  });
  for(var k=0;k<3;k++){maskLoaded[k]=BigInt('0x'+CFG['tofMask'+k]);maskEdit[k]=maskLoaded[k]}
  $('maskSave').disabled=false;$('maskSave').title='';
  document.querySelectorAll('.fsrUse').forEach(function(c){c.checked=!!(CFG.fsrEnabled&(1<<+c.dataset.u))});
  flFill();
  var L=$('limTxt');if(L)L.textContent=CFG.limitsValid?(CFG.openZero+' .. '+CFG.closeLimit):'not set';
}
function loadCfg(){return getJ('/api/config').then(function(j){CFG=j;fillForms();drawTof()}).catch(function(){})}
$('maskSave').disabled=true;$('maskSave').title='config not loaded';
setInterval(function(){if(!CFG)loadCfg()},2000);
function saveForm(id,extra){
  var o={},n=0;
  document.querySelectorAll('#'+id+' [data-k]').forEach(function(el){
    var cur=el.type==='checkbox'?(el.checked?'1':'0'):el.value;
    if(cur!==el.dataset.o&&cur!==''){o[el.dataset.k]=cur;n++}
  });
  if(extra)Object.keys(extra).forEach(function(k){o[k]=extra[k];n++});
  if(!n){toast('No changes');return}
  post('/api/config',o).then(function(j){if(j){toast('Saved (queued)');setTimeout(loadCfg,500)}});
}
$('svSave').onclick=function(){saveForm('svCfg')};
$('swSave').onclick=function(){saveForm('swCfg')};
$('objSave').onclick=function(){saveForm('objCfg')};
$('objOv').onchange=function(){post('/api/object/override?cls='+this.value)};
$('objDel').onchange=function(){post('/api/object/delicate?on='+(this.checked?1:0))};
$('objAppr').onchange=function(){post('/api/object/approachOnly?on='+(this.checked?1:0))};
$('objBg').onclick=function(){if(confirm('Capture background: gripper empty and nothing in the SEN0628 view?'))post('/api/object/captureBackground')};
function renderObject(){var O=S.object;if(!O)return;
  try{OZ=(O.zones||['0','0','0']).map(function(h){return BigInt('0x'+h)})}catch(e){OZ=[0n,0n,0n]}
  var badges='';
  if(O.delicate)badges+='<span class="pill on" style="background:#c0392b;color:#fff">DELICATE</span> ';
  if(O.override&&O.override!=='auto')badges+='<span class="pill on" style="background:#8e44ad;color:#fff">CLASS: '+esc(O.override.toUpperCase())+' (forced)</span> ';
  if(O.approachOnly)badges+='<span class="pill on" style="background:#d68910;color:#fff">APPROACH ONLY</span> ';
  var meas=O.valid?('width '+fx(O.width,0)+' mm, range '+fx(O.range,0)+' mm, height '+fx(O.height,0)+' mm, '+esc(O.shape)):'no object measured';
  var det=O.clsDetected?('detected <b>'+esc(O.clsDetected)+'</b>'):'detected -';
  var eff=O.cls?('grasp as <b>'+esc(O.cls)+'</b>'):('cannot grasp: '+esc(O.planWhy||'-'));
  var st=S.state,ph='';
  if(st==='PRE_TOUCH')ph='PRE_TOUCH step: <b>'+esc(O.step)+'</b>'+(O.approach?' (goal '+O.goal+')':'');
  var pat='';
  if(st==='PRE_TOUCH'||st==='GRASP'||st==='MONITOR'){
    pat='contact: <b>'+esc(O.pattern)+'</b> '+(O.pattern==='none'?'':(O.patternOk?'&#10003;':'&#9888; unexpected for '+esc(O.latched)))+
      ' <span class="small">('+(O.first||[]).map(function(f,i){return FNAME[i]+': '+esc(f)}).join(', ')+')</span>'}
  var html=badges+'<div>'+meas+'</div><div>'+det+' &middot; '+eff+'</div>'+(ph?'<div>'+ph+'</div>':'')+(pat?'<div>'+pat+'</div>':'');
  $('cObj').innerHTML=html;$('objLive').innerHTML=html;
  var ov=$('objOv');if(document.activeElement!==ov)ov.value=O.override||'auto';
  $('objDel').checked=!!O.delicate;$('objAppr').checked=!!O.approachOnly;
}
$('fsrSave').onclick=function(){saveForm('fsrCfg')};
$('tofSave').onclick=function(){saveForm('tofCfg')};
$('maskSave').onclick=function(){
  if(!CFG)return;
  var o={};for(var k=0;k<3;k++)if(CFG&&maskEdit[k]!==maskLoaded[k])o['tofMask'+k]=hex16(maskEdit[k]);
  if(!Object.keys(o).length){toast('No changes');return}
  post('/api/config',o).then(function(j){if(j){toast('Mask saved (queued)');setTimeout(loadCfg,500)}});
};

/* ---------- Servo ---------- */
function renderServo(){
  var s=S.servo||{},L=S.limits||{},rel=(L.valid&&s.position!=null)?(s.position-L.openZero)*(CFG&&CFG.closePositive===false?-1:1):null,
      rows=[['Online',s.online?'yes':'NO'],['Age',s.ageMs+' ms'],['Position',s.position],
      ['OPEN zero (saved)',L.valid?L.openZero:'not set'],['Close limit (saved)',L.valid?L.closeLimit:'not set'],
      ['Position from OPEN zero',rel===null?'-':rel],['Speed',s.speed],['Load',fx(s.load,1)],['Current',s.current+' ('+Math.round(s.current*6.5)+' mA)'],['Voltage',fx(s.voltage,1)+' V'],['Temp',s.temp+' C'],['Status',s.status+' / reply '+s.replyStatus+' ('+esc(s.statusText)+')'],['Mode',s.mode===null?'?':s.mode],['Full angle',s.fullAngle?'yes':'NO'],['Last error',esc(s.lastError||'-')]];
  var h='';rows.forEach(function(r){h+='<tr><th>'+r[0]+'</th><td>'+r[1]+'</td></tr>'});
  $('svTbl').innerHTML=h;
}
var jogTimer=0,jogging=false,jogBusy=false;
/* Stop is sent at once, even while a jog POST is pending, and retried once on failure. */
function sendStop(){post('/api/servo/stop').then(function(j){if(!j)post('/api/servo/stop')})}
function jogStop(){
  if(!jogTimer&&!jogging)return;
  clearInterval(jogTimer);jogTimer=0;jogging=false;
  sendStop();
}
function jogStart(dir){
  var sp=Math.max(1,Math.min(61,parseInt($('jogSpd').value,10)||20)),ign=$('jogIgn').checked?1:0;
  var send=function(){
    if(jogBusy||!jogging)return;           /* one jog POST in flight at a time */
    jogBusy=true;
    post('/api/servo/jog',{speed:dir*sp,ignoreLimits:ign}).then(function(){
      jogBusy=false;
      if(!jogging)sendStop();              /* released while pending: the jog may have landed after the stop */
    });
  };
  jogging=true;send();clearInterval(jogTimer);jogTimer=setInterval(send,150);
}
[['jogC',1],['jogO',-1]].forEach(function(p){
  var el=$(p[0]);
  el.addEventListener('pointerdown',function(e){e.preventDefault();try{el.setPointerCapture(e.pointerId)}catch(x){}jogStart(p[1])});
  ['pointerup','pointerleave','pointercancel'].forEach(function(ev){el.addEventListener(ev,jogStop)});
  el.addEventListener('contextmenu',function(e){e.preventDefault()});
});
window.addEventListener('blur',jogStop);
document.addEventListener('visibilitychange',function(){if(document.hidden)jogStop()});
function setLim(u){var H=(S&&S.home)||{};
  if(H.touchValid&&!H.homed){if(!confirm('Not homed. Saving a limit now starts a new position frame: the touch reference is dropped and the calibration sweep must be run again. Usually press Home instead. Continue?'))return;u+='?newFrame=1'}
  post(u)}
$('setOpen').onclick=function(){setLim('/api/servo/setOpen')};
$('setClose').onclick=function(){setLim('/api/servo/setClose')};
$('scan').onclick=function(){post('/api/servo/scan')};
$('posMode').onclick=function(){if(confirm('Put the servo in position mode 0 (saved in the servo)? Torque goes off, the position count may reset and the limits are cleared.'))post('/api/servo/positionMode')};
$('calMid').onclick=function(){if(confirm('Make the present position the servo middle (Waveshare calibration)? Torque goes off, all positions shift and the limits are cleared.'))post('/api/servo/calibrateMiddle')};
$('clearLim').onclick=function(){if(confirm('Clear OPEN zero and close limit? You will need to set both again.'))post('/api/servo/clearLimits')};
/* Ignore-limits is one-shot: it switches itself off after each jog release. */
['jogC','jogO'].forEach(function(id){$(id).addEventListener('pointerup',function(){setTimeout(function(){$('jogIgn').checked=false},0)})});

/* ---------- FSR ---------- */
var CAL=null;
function fingerOf(i){for(var f=0;f<3;f++)if(FMAP[f][0]===i||FMAP[f][1]===i)return f+1;return '-'}
function mkFsr(){
  var h='<tr><th>Ch</th><th>Finger</th><th>raw</th><th>mV</th><th>ohm</th><th>N</th><th>Cal</th><th>Used</th><th></th></tr>';
  for(var i=0;i<6;i++)h+='<tr><td>'+FSR_PRINT[i]+'</td><td>'+fingerOf(i)+'</td><td id="fr'+i+'"></td><td id="fm'+i+'"></td><td id="fo'+i+'"></td><td id="ff'+i+'"></td><td id="fc'+i+'"></td><td><input type="checkbox" class="fsrUse" data-u="'+i+'" checked></td><td><button class="b" data-z="'+i+'" style="padding:4px 10px">Zero</button></td></tr>';
  $('fsrTbl').innerHTML=h;
  /* Used checkboxes: bit i of fsrEnabled. Saved at once; the FSM ignores unticked channels. */
  $('fsrTbl').onchange=function(e){
    if(!e.target.classList.contains('fsrUse'))return;
    if(!CFG){e.target.checked=!e.target.checked;toast('config not loaded');return}
    var m=0;document.querySelectorAll('.fsrUse').forEach(function(c){if(c.checked)m|=1<<+c.dataset.u});
    post('/api/config',{fsrEnabled:m}).then(function(j){if(j)toast('Saved (queued)');setTimeout(loadCfg,500)});
  };
  $('fsrTbl').onclick=function(e){var z=e.target.dataset&&e.target.dataset.z;if(z!==undefined&&z!=='')post('/api/fsr/zero',{id:z}).then(function(j){if(j){toast('Zeroed');loadCal()}})};
}
/* ---------- Home and calibration sweep ---------- */
var swWas=false;
function renderHome(){var H=S.home||{};
  ['homed','homed2'].forEach(function(id){var e=$(id);e.textContent=H.homing?'HOMING...':H.sweeping?'SWEEPING...':H.homed?'HOMED':'NOT HOMED';e.className='pill '+(H.homed?'on':'off')});
  document.querySelectorAll('.homeMsg').forEach(function(e){e.textContent=H.msg||''});
  $('estTxt').textContent='SEN0628 estimate: '+(H.estimate?('turn '+H.estimate.turn+', position '+H.estimate.pos):(H.homed?'- (homed)':'none (fingers not visible, or no sweep)'));
  $('swTxt').textContent='Calibration sweep: '+(H.sweeping?('recording, '+H.sweepN+' samples'):(H.sweepLoaded?'stored':'none yet - run it once'));
  if(swWas&&!H.sweeping)loadSweep();swWas=!!H.sweeping;}
function loadSweep(){getJ('/api/sweep').then(function(j){SW=j;FZ=(j.fingerZones||['0','0','0']).map(function(h){return BigInt('0x'+h)});drawSweep()}).catch(function(){})}
var SWG=null,swSel=-1,swDrag=false,swMoved=false;
function drawSweep(){var c=$('swChart');if(!c||!c.clientWidth)return;var dpr=window.devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
  c.width=w*dpr;c.height=h*dpr;var g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,h);
  g.fillStyle='#8a96a3';g.font='11px system-ui';
  if(!SW||!SW.n){g.fillText('no sweep stored',10,20);return}
  var P=SW.positions,O=SW.opening,x0=Math.min(SW.openZero,P[0],P[P.length-1]),x1=Math.max(SW.openZero,P[0],P[P.length-1]),
      ymax=Math.max.apply(null,O.concat([175])),L=40,B=20,X=function(v){return L+(w-L-8)*(v-x0)/((x1-x0)||1)},Y=function(v){return h-B-(h-B-8)*v/ymax};
  SWG={X:X,Y:Y,h:h,B:B,ymax:ymax};
  g.strokeStyle='#26313d';for(var t=0;t<=4;t++){var v=ymax*t/4,y=Y(v);g.beginPath();g.moveTo(L,y);g.lineTo(w-8,y);g.stroke();g.fillText(v.toFixed(0),4,y+4)}
  g.fillText(String(x0),L,h-4);g.fillText(String(x1),w-60,h-4);
  var fs=SW.firstSeen>=0?SW.firstSeen:P.length;g.strokeStyle='#4ea1ff';g.lineWidth=2;
  g.setLineDash([5,4]);g.beginPath();P.forEach(function(p,i){if(i>fs)return;var x=X(p),y=Y(O[i]);i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();
  g.setLineDash([]);g.beginPath();P.forEach(function(p,i){if(i<fs)return;var x=X(p),y=Y(O[i]);i>fs?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();g.lineWidth=1;
  g.fillStyle='#8a96a3';g.fillText('dashed: fingers not seen (estimated)',L+4,h-B-6);
  var A=SW.auto||O,E=SW.edited||[];
  P.forEach(function(p,i){var x=X(p);
    if(E[i]){g.strokeStyle='#5d6876';g.beginPath();g.moveTo(x,Y(A[i]));g.lineTo(x,Y(O[i]));g.stroke();
      g.fillStyle='#5d6876';g.beginPath();g.arc(x,Y(A[i]),2,0,7);g.fill();g.fillStyle='#ff9f1c'}
    else g.fillStyle='rgba(78,161,255,.6)';
    g.beginPath();g.arc(x,Y(O[i]),i===swSel?5:(E[i]?3.5:2),0,7);g.fill();
    if(i===swSel){g.strokeStyle='#fff';g.lineWidth=1.5;g.stroke();g.lineWidth=1}});
  swSelTxt();
  g.fillStyle='#9be38b';g.fillText('OPEN',X(SW.openZero)+3,12);g.fillStyle='#ffb04e';g.fillText('touch',X(SW.touchRef)-34,12);
  g.strokeStyle='#9be38b';g.beginPath();g.moveTo(X(SW.openZero),8);g.lineTo(X(SW.openZero),h-B);g.stroke();
  g.strokeStyle='#ffb04e';g.beginPath();g.moveTo(X(SW.touchRef),8);g.lineTo(X(SW.touchRef),h-B);g.stroke();}
function swSelTxt(){var t=$('swSel');if(!SW||swSel<0||swSel>=SW.n){t.textContent='No point selected';return}
  var a=(SW.auto||SW.opening)[swSel],o=SW.opening[swSel];
  t.textContent='Point '+swSel+' at '+SW.positions[swSel]+': analysed '+fx(a,1)+' mm, now '+fx(o,1)+' mm'+((SW.edited||[])[swSel]?' (edited)':'');
  if(document.activeElement!==$('swMm'))$('swMm').value=fx(o,1)}
function swEdit(i,mm){post('/api/sweep/opening?i='+i+(mm===null?'':'&mm='+mm)).then(function(){setTimeout(loadSweep,600)})}
(function(){var c=$('swChart');
  function near(ev){if(!SW||!SW.n||!SWG)return -1;var r=c.getBoundingClientRect(),x=ev.clientX-r.left,b=-1,bd=12;
    SW.positions.forEach(function(p,i){var d=Math.abs(SWG.X(p)-x);if(d<bd){bd=d;b=i}});return b}
  function mmAt(ev){var r=c.getBoundingClientRect(),y=ev.clientY-r.top,v=(SWG.h-SWG.B-y)/(SWG.h-SWG.B-8)*SWG.ymax;return Math.max(0,Math.min(250,Math.round(v*10)/10))}
  c.addEventListener('pointerdown',function(ev){var i=near(ev);if(i<0)return;swSel=i;swDrag=true;swMoved=false;c.setPointerCapture(ev.pointerId);drawSweep()});
  c.addEventListener('pointermove',function(ev){if(!swDrag)return;swMoved=true;SW.opening[swSel]=mmAt(ev);drawSweep()});
  c.addEventListener('pointerup',function(){if(!swDrag)return;swDrag=false;if(swMoved)swEdit(swSel,SW.opening[swSel])});
})();
$('swSet').onclick=function(){var v=parseFloat($('swMm').value);if(swSel<0||isNaN(v)){toast('Select a point first');return}swEdit(swSel,v)};
$('swUndo').onclick=function(){if(swSel<0){toast('Select a point first');return}swEdit(swSel,null)};
$('swClr').onclick=function(){if(confirm('Clear every curve edit and use the analysed curve again?'))swEdit(-1,null)};
['bHome','bHome2'].forEach(function(id){$(id).onclick=function(){if(confirm('Home: is the gripper empty? The fingers close slowly until a fingertip touches, then open.'))post('/api/servo/home')}});
$('bSweep').onclick=function(){if(confirm('Calibration sweep: gripper empty and nothing in the SEN0628 view? Takes a few minutes; replaces the stored sweep only if it completes.'))post('/api/servo/sweep')};
loadSweep();

/* ---------- FSR filter chart (ring buffer of the last 10 s of status polls) ---------- */
var FL_N=50,flHist=[];   /* 50 polls x 200 ms = 10 s */
function flShowParams(){var k=+$('flKind').value;
  $('flWinL').style.display=(k===1||k===2)?'':'none';$('flAlphaL').style.display=k===3?'':'none';
  $('flQL').style.display=k===4?'':'none';$('flRL').style.display=k===4?'':'none';}
function flFill(){if(!CFG)return;$('flKind').value=CFG.fsrFilter;$('flWin').value=CFG.fsrFilterWindow;
  $('flAlpha').value=CFG.fsrFilterAlpha;$('flQ').value=CFG.fsrKalmanQ;$('flR').value=CFG.fsrKalmanR;flShowParams();}
function flPush(){var f=S.fsr;if(!f||!f.forceRaw)return;
  flHist.push({r:f.forceRaw.slice(),f:f.force.slice(),l:f.limitForce||0});if(flHist.length>FL_N)flHist.shift();}
function flDraw(){var c=$('flChart');if(!c||!c.clientWidth)return;var dpr=window.devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
  c.width=w*dpr;c.height=h*dpr;var g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,h);
  var ch=+$('flCh').value||0,mx=0.5;flHist.forEach(function(p){[p.r[ch],p.f[ch]].forEach(function(v){if(v!==null&&v>mx)mx=v})});
  mx*=1.15;var L=36,B=18,X=function(i){return L+(w-L-6)*i/(FL_N-1)},Y=function(v){return h-B-(h-B-6)*v/mx};
  g.strokeStyle='#26313d';g.fillStyle='#8a96a3';g.font='11px system-ui';g.lineWidth=1;
  for(var t=0;t<=4;t++){var v=mx*t/4,y=Y(v);g.beginPath();g.moveTo(L,y);g.lineTo(w-6,y);g.stroke();g.fillText(v.toFixed(v<2?2:1),2,y+4)}
  [['r','#8a96a3',1],['f','#4ea1ff',2]].forEach(function(s){g.strokeStyle=s[1];g.lineWidth=s[2];g.beginPath();var on=false;
    flHist.forEach(function(p,i){var v=p[s[0]][ch];if(v===null){on=false;return}var x=X(i+FL_N-flHist.length),y=Y(v);if(on)g.lineTo(x,y);else{g.moveTo(x,y);on=true}});g.stroke()});
  var pk=0;flHist.forEach(function(p){if(p.l>pk)pk=p.l});var d=S.diag||{};
  if(S.fsr)$('flLimit').textContent='Force-limit input (max of raw and filtered, used pads): '+fx(S.fsr.limitForce,2)+' N, peak last 10 s '+fx(pk,2)+' N'+(CFG?' (limit '+fx(CFG.maxForceN,1)+' N)':'')+' \u00b7 FSR sample pass max '+fx((d.fsrSampleUsMax||0)/1000,1)+' ms (5 s)';
  var f=S.fsr;$('flDelay').textContent=f&&f.filterDelayMs!=null?' Delay \u2248 '+fx(f.filterDelayMs,0)+' ms at 100 Hz':'';}
$('flKind').onchange=flShowParams;
$('flSave').onclick=function(){if(!CFG){toast('config not loaded');return}
  var k=+$('flKind').value,o={fsrFilter:k};
  if(k===1||k===2)o.fsrFilterWindow=$('flWin').value;if(k===3)o.fsrFilterAlpha=$('flAlpha').value;
  if(k===4){o.fsrKalmanQ=$('flQ').value;o.fsrKalmanR=$('flR').value}
  post('/api/config',o).then(function(j){if(j){toast('Filter applied (queued)');flHist=[];setTimeout(loadCfg,500)}});};
(function(){var h='';for(var i=0;i<6;i++)h+='<option value="'+i+'">FSR '+FSR_PRINT[i]+' ('+fingerOf(i)+')</option>';$('flCh').innerHTML=h;})();
$('flCh').onchange=flDraw;

function renderFsr(){
  flDraw();
  var f=S.fsr;if(!f)return;
  for(var i=0;i<6;i++){
    $('fr'+i).textContent=f.raw[i];$('fm'+i).textContent=fx(f.mv[i],0);
    $('fo'+i).textContent=f.ohms[i]===null?'-':fx(f.ohms[i],0);
    $('ff'+i).textContent=f.calibrated[i]?fx(f.force[i],2):'-';
    $('fc'+i).textContent=f.calibrated[i]?'yes':'NO';
  }
}
function loadCal(){getJ('/api/fsr/cal').then(function(j){CAL=j.cal;
  var h='';CAL.forEach(function(c){
    h+='<div class="row fcal" data-id="'+c.id+'"><b style="width:46px">Ch '+FSR_PRINT[c.id]+'</b>';
    ['r','f1','o1','f2','o2','z'].forEach(function(k){h+='<label class="f">'+k+'<input type="number" step="any" data-c="'+k+'" value="'+c[k]+'"></label>'});
    h+='<button class="b" data-s="'+c.id+'" style="padding:6px 12px">Save</button><span class="small">'+(c.valid?'valid':'INVALID')+'</span></div>';
  });
  $('calBox').innerHTML=h}).catch(function(){})}
$('calBox').onclick=function(e){
  var id=e.target.dataset&&e.target.dataset.s;if(id===undefined||id==='')return;
  var o={id:id};e.target.parentNode.querySelectorAll('[data-c]').forEach(function(el){o[el.dataset.c]=el.value});
  post('/api/fsr/cal',o).then(function(j){if(j){toast('Calibration saved');loadCal()}});
};

/* ---------- polling ---------- */
function ok(){fails=0;$('banner').style.display='none'}
function bad(){if(++fails>=3)$('banner').style.display='block'}
function pollStatus(){
  if(stPend)return;stPend=true;
  getJ('/api/status').then(function(j){S=j;ok();render()}).catch(bad).then(function(){stPend=false});
}
function pollTof(){
  if(tfPend||(tab!=='comb'&&tab!=='tof'))return;tfPend=true;
  getJ('/api/tof').then(function(j){T=j;drawTof()}).catch(function(){}).then(function(){tfPend=false});
}

/* ---------- Recordings (phase 4a) ---------- */
var TLOG={next:null},PRE=[],PRET=[],LOG={next:null,boot:null},REC=null,MAN=null,RECS=[],DB=null,logBusy=false,logRows=0,logT0=Date.now(),recSeqN=0;
function idbOpen(){return new Promise(function(res){try{var q=indexedDB.open('sensegrip',1);
  q.onupgradeneeded=function(){var d=q.result;d.createObjectStore('meta');d.createObjectStore('recs',{keyPath:'id'})};
  q.onsuccess=function(){res(q.result)};q.onerror=function(){res(null)}}catch(e){res(null)}})}
function idbPut(st,v,k){if(!DB)return;try{var o=DB.transaction(st,'readwrite').objectStore(st);k===undefined?o.put(v):o.put(v,k)}catch(e){}}
function idbDel(id){if(!DB)return;try{DB.transaction('recs','readwrite').objectStore('recs').delete(id)}catch(e){}}
function idbGet(st,k){return new Promise(function(res){if(!DB)return res(null);try{var q=DB.transaction(st).objectStore(st).get(k);q.onsuccess=function(){res(q.result||null)};q.onerror=function(){res(null)}}catch(e){res(null)}})}
function idbAll(){return new Promise(function(res){if(!DB)return res([]);try{var q=DB.transaction('recs').objectStore('recs').getAll();q.onsuccess=function(){res(q.result||[])};q.onerror=function(){res([])}}catch(e){res([])}})}
function stamp(t){var d=new Date(t),p=function(n){return(n<10?'0':'')+n};return d.getFullYear()+p(d.getMonth()+1)+p(d.getDate())+'-'+p(d.getHours())+p(d.getMinutes())+p(d.getSeconds())}
function newRec(kind){recSeqN++;return{id:Date.now()+Math.random(),kind:kind,n:recSeqN,start:Date.now(),rows:[],tof:[],marks:0,cls:'',width:'',final:'',gap:false,endAt:null,sawMon:false,open:true,
  fw:(S&&S.diag&&S.diag.fw)||'',cfg:CFG?JSON.parse(JSON.stringify(CFG)):null}}
function recName(r){return (r.kind==='grasp'?'grasp_':'manual_')+r.n+(r.cls?'_'+r.cls:'')+'_'+stamp(r.start)+'.csv'}
function closeRec(r,why){r.open=false;if(why)r.final=r.final||why;idbPut('recs',r);if(RECS.indexOf(r)<0)RECS.push(r);renderRecs()}
function onTofLine(line){var rs=+line.split(',')[2];
  PRET.push(line);if(PRET.length>40)PRET.shift();               // ~2 s pre-roll
  if(REC&&rs>=REC.firstSeq)REC.tof.push(line);
  if(MAN&&rs>=MAN.firstSeq)MAN.tof.push(line)}
function onLogLine(line){
  var f=line.split(','),seq=+f[0],st=f[2];logRows++;
  PRE.push(line);if(PRE.length>100)PRE.shift();                  // 1 s pre-roll
  if(f[34]&&f[34].slice(0,5)==='mark:'){if(REC)REC.marks++;if(MAN)MAN.marks++}
  if(REC){REC.rows.push(line);
    if(!REC.cls&&f[32])REC.cls=f[32];if(!REC.width&&f[26])REC.width=f[26];
    if(st==='MONITOR')REC.sawMon=true;
    if(REC.endAt===null&&(st==='IDLE'||st==='FAULT')){REC.endAt=seq+100;REC.final=st==='FAULT'?'FAULT':(REC.sawMon?'grasped':'aborted')}
    if(REC.endAt!==null&&seq>=REC.endAt){closeRec(REC);REC=null}}
  else if(st==='START'){REC=newRec('grasp');REC.rows=PRE.slice();REC.firstSeq=+REC.rows[0].split(',')[0];
    REC.tof=PRET.filter(function(t){return +t.split(',')[2]>=REC.firstSeq});RECS.push(REC)}
  if(MAN){if(MAN.firstSeq===undefined)MAN.firstSeq=seq;MAN.rows.push(line)}
}
function tofPoll(){var from=TLOG.next===null?0:TLOG.next;
  return fetch('/api/log/tof?from='+from+'&max=60',{cache:'no-store'}).then(function(r){if(!r.ok)return;
    var nx=+r.headers.get('X-Log-Next'),boot=r.headers.get('X-Log-Boot');
    return r.text().then(function(t){if(LOG.boot!==null&&boot!==LOG.boot){TLOG.next=null;return}
      t.split('\n').forEach(function(l){if(l&&l.slice(0,4)!=='seq,')onTofLine(l)});TLOG.next=nx})}).catch(function(){})}
function logPoll(){if(logBusy)return;logBusy=true;
  var from=LOG.next===null?0:LOG.next;
  tofPoll().then(function(){return fetch('/api/log?from='+from+'&max=600',{cache:'no-store'})}).then(function(r){
    if(!r.ok)throw 0;
    var nx=+r.headers.get('X-Log-Next'),old=+r.headers.get('X-Log-Oldest'),boot=r.headers.get('X-Log-Boot');
    return r.text().then(function(t){
      if(LOG.boot!==null&&boot!==LOG.boot){if(REC){REC.gap=true;closeRec(REC,'gripper restarted');REC=null}if(MAN)MAN.gap=true;LOG.next=null;LOG.boot=boot;return}
      LOG.boot=boot;
      if(LOG.next!==null&&old>LOG.next){if(REC)REC.gap=true;if(MAN)MAN.gap=true}
      t.split('\n').forEach(function(l){if(l&&l.slice(0,4)!=='seq,')onLogLine(l)});
      LOG.next=nx;idbPut('meta',{next:LOG.next,boot:LOG.boot,tnext:TLOG.next},'log');
    })}).catch(function(){}).then(function(){logBusy=false;
      var dt=(Date.now()-logT0)/1000;if(dt>=2){$('recStat').textContent='collector: '+Math.round(logRows/dt)+' rows/s'+(REC?' · recording grasp ('+REC.rows.length+' rows)':'')+(MAN?' · manual ('+MAN.rows.length+' rows)':'');logRows=0;logT0=Date.now()}})}
function csvOf(r){var c=r.cfg||{},k=r.cls,L=[];
  L.push('# SenseGrip recording '+recName(r));L.push('# firmware: '+(r.fw||'?'));
  L.push('# started: '+new Date(r.start).toISOString()+' | rows: '+r.rows.length+' | result: '+(r.final||'-')+(r.gap?' | GAP: rows missing':''));
  if(k&&c[k+'ForceN']!==undefined)L.push('# class profile '+k+': stand-off '+c[k+'StandoffMm']+' mm, target '+c[k+'ForceN']+' N, max current '+c[k+'MaxCurrent']+' x6.5 mA, grasp speed '+c[k+'GraspSpeed']);
  L.push('# FSR filter: '+['none','moving average','median','EMA','Kalman'][c.fsrFilter||0]+' window '+c.fsrFilterWindow+' alpha '+c.fsrFilterAlpha+' | contact force '+c.contactForceN+' N | max force '+c.maxForceN+' N');
  L.push('# units: forces N (f=filtered, r=raw, FSR1..6), pos servo counts, current mA, load %, sizes mm');
  return L.join('\n')+'\n'+LOG_HEADER+'\n'+r.rows.join('\n')+'\n'}
var LOG_HEADER='seq,t_ms,state,step,armed,pos,speed,current_mA,load_pct,f1,f2,f3,f4,f5,f6,r1,r2,r3,r4,r5,r6,c1,c2,c3,tof_seq,obj_valid,width_mm,range_mm,height_mm,rad1,rad2,rad3,cls,shape,event';
var TOF_HEADER=null;
function tofHeader(){if(TOF_HEADER)return TOF_HEADER;var h='seq,t_ms,row_seq,pos,valid';for(var s=1;s<=3;s++)for(var z=0;z<64;z++)h+=',s'+s+'_z'+(z<10?'0':'')+z;return TOF_HEADER=h}
function save(name,text){var a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type:'text/csv'}));a.download=name;document.body.appendChild(a);a.click();setTimeout(function(){URL.revokeObjectURL(a.href);a.remove()},1000)}
function dl(r){save(recName(r),csvOf(r));
  if(r.tof&&r.tof.length)setTimeout(function(){save(recName(r).replace(/\.csv$/,'_tof.csv'),
    '# SenseGrip raw SEN0628 frames for '+recName(r)+' | mm per zone, raw zone index (row-major as read), 0 or >=4000 = no target | row_seq links to seq in the main file\n'+tofHeader()+'\n'+r.tof.join('\n')+'\n')},400)}
function renderRecs(){var l=$('recList');if(!RECS.length){l.textContent='No recordings yet';return}
  var h='<table><tr><th>Name</th><th>s</th><th>rows</th><th>class</th><th>width</th><th>result</th><th></th></tr>';
  RECS.slice().reverse().forEach(function(r){var i=RECS.indexOf(r),d=r.rows.length?((+r.rows[r.rows.length-1].split(',')[1]-(+r.rows[0].split(',')[1]))/1000).toFixed(1):'0';
    h+='<tr><td>'+esc(recName(r))+(r.gap?' <b style="color:#ffb04e">gap</b>':'')+(r.open?' (recording)':'')+(r.marks?' · '+r.marks+' marks':'')+(r.tof&&r.tof.length?' · '+r.tof.length+' frames':'')+'</td><td>'+d+'</td><td>'+r.rows.length+'</td><td>'+esc(r.cls||'-')+'</td><td>'+esc(r.width||'-')+'</td><td>'+esc(r.final||'-')+'</td>'+
      '<td><button class="b" data-dl="'+i+'">CSV</button> <button class="b" data-rm="'+i+'">Delete</button></td></tr>'});
  l.innerHTML=h+'</table>';
  l.querySelectorAll('[data-dl]').forEach(function(b){b.onclick=function(){dl(RECS[+b.dataset.dl])}});
  l.querySelectorAll('[data-rm]').forEach(function(b){b.onclick=function(){var r=RECS[+b.dataset.rm];if(r.open){toast('Still recording');return}if(!confirm('Delete '+recName(r)+'?'))return;idbDel(r.id);RECS.splice(+b.dataset.rm,1);renderRecs()}})}
$('recMan').onclick=function(){if(MAN){closeRec(MAN,'manual stop');MAN=null;this.textContent='Start manual recording'}else{MAN=newRec('manual');RECS.push(MAN);this.textContent='Stop manual recording';renderRecs()}};
$('recAll').onclick=function(){var k=0;RECS.forEach(function(r){if(!r.open)setTimeout(function(){dl(r)},900*k++)})};
function mark(t){post('/api/log/mark?type='+t).then(function(j){if(j)toast('Mark: '+t)})}
document.querySelectorAll('[data-mark]').forEach(function(b){b.onclick=function(){mark(b.dataset.mark)}});
document.addEventListener('keydown',function(ev){var tg=(ev.target&&ev.target.tagName)||'';if(/INPUT|SELECT|TEXTAREA/.test(tg)||ev.ctrlKey||ev.metaKey||ev.altKey||ev.repeat)return;
  var k={p:'pull',t:'twist',s:'slide',o:'other',e:'end'}[ev.key.toLowerCase()];if(k){ev.preventDefault();mark(k)}});
$('recClr').onclick=function(){if(!confirm('Delete every recording stored in this browser?'))return;RECS.filter(function(r){return !r.open}).forEach(function(r){idbDel(r.id)});RECS=RECS.filter(function(r){return r.open});renderRecs()};
setInterval(function(){[REC,MAN].forEach(function(r){if(r)idbPut('recs',r)})},5000);
idbOpen().then(function(d){DB=d;return Promise.all([idbGet('meta','log'),idbAll()])}).then(function(v){
  var m=v[0];if(m){LOG.next=m.next;LOG.boot=m.boot;TLOG.next=m.tnext===undefined?null:m.tnext}
  RECS=(v[1]||[]).sort(function(a,b){return a.start-b.start});
  RECS.forEach(function(r){recSeqN=Math.max(recSeqN,r.n||0);if(r.open){if(r.kind==='grasp'&&!REC)REC=r;else{r.open=false;r.final=r.final||'page closed';idbPut('recs',r)}}});
  renderRecs();setInterval(logPoll,250)});
buildForms();mkFingers();mkFsr();loadCfg();
setInterval(pollStatus,200);setInterval(pollTof,200);
pollStatus();pollTof();
</script></body></html>)HTML";
