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
header{display:flex;align-items:center;gap:12px;padding:8px 12px;background:var(--pn);border-bottom:1px solid var(--bd)}
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
#estop{width:100%;padding:22px;font-size:26px;font-weight:900;background:#c4161c;border:3px solid #ff6b6f;color:#fff;border-radius:12px;margin-top:8px;cursor:pointer}
.btns{display:flex;gap:8px;flex-wrap:wrap}.btns .b{flex:1;min-width:90px}
canvas{width:100%;aspect-ratio:1;background:#0c1117;border:1px solid var(--bd);border-radius:6px;display:block}
.hm{text-align:center}.hm .small{margin-top:4px}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{padding:4px 6px;border-bottom:1px solid var(--bd);text-align:left}
input[type=number],input[type=text]{background:#0c1117;color:var(--tx);border:1px solid var(--bd);border-radius:5px;padding:6px;width:100%;font-size:14px}
label.f{display:block;font-size:13px;color:var(--mu)}
.cfg{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px;margin:8px 0}
.jog{user-select:none;-webkit-user-select:none;touch-action:none}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:6px 0}
.fcal input{width:64px;padding:4px}
</style></head><body>
<div id="banner">Connection lost - retrying...</div>
<header><h1>SenseGrip</h1><span id="hdrState" class="pill">...</span></header>
<nav id="tabs"><button data-t="comb" class="on">Combined</button><button data-t="servo">Servo</button><button data-t="fsr">FSR</button><button data-t="tof">SEN0628</button></nav>
<main>
<section class="tab on" id="t-comb">
 <div class="grid">
  <div>
   <div class="panel"><div id="badge">...</div>
    <div id="faultBox"></div>
    <div class="row"><span class="small">Armed:</span><span id="armed" class="pill off">DISARMED</span><span class="small" id="reason"></span></div></div>
   <div class="panel"><h2>Blockers</h2><div id="blk"></div></div>
   <div class="panel"><h2>Last action</h2><div id="last" class="small">-</div></div>
   <div class="panel"><div class="btns">
     <button class="b pri" id="bArm">Arm</button><button class="b" id="bOpen">Open</button><button class="b" id="bReset">Reset</button></div>
     <button id="estop">E-STOP</button></div>
  </div>
  <div>
   <div class="panel"><h2>Finger force</h2><div id="fingers"></div>
    <h2 style="margin-top:12px">Position</h2><div class="bar"><i id="posBar"></i><span id="posTxt"></span></div></div>
   <div class="panel"><h2>ToF (SEN0628)</h2><div class="grid3" id="cHm"></div></div>
  </div>
 </div>
</section>

<section class="tab" id="t-servo">
 <div class="grid">
  <div class="panel"><h2>Telemetry</h2><table id="svTbl"></table></div>
  <div class="panel"><h2>Manual</h2>
   <div class="row"><label class="f">Jog speed (1-61)<input type="number" id="jogSpd" min="1" max="61" value="20"></label></div>
   <div class="btns"><button class="b jog" id="jogC">Jog Close (hold)</button><button class="b jog" id="jogO">Jog Open (hold)</button></div>
   <div class="btns" style="margin-top:8px"><button class="b" id="setOpen">Set OPEN zero here</button><button class="b" id="setClose">Set close limit here</button></div>
   <div class="btns" style="margin-top:8px"><button class="b" id="scan">Scan</button></div>
   <p class="small">Jog is only accepted while IDLE and disarmed. Limits: <span id="limTxt">-</span></p></div>
 </div>
 <div class="panel"><h2>Config</h2><div class="cfg" id="svCfg"></div><button class="b pri" id="svSave">Save</button></div>
</section>

<section class="tab" id="t-fsr">
 <div class="panel"><h2>Channels</h2><div style="overflow-x:auto"><table id="fsrTbl"></table></div></div>
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
function drawHm(cv,sn,mask,nums){
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
  var st=S.state||'?';
  var b=$('badge');b.textContent=st;b.className='st-'+st;
  var h=$('hdrState');h.textContent=st;h.className='pill st-'+st;
  var fb=$('faultBox');
  if(st==='FAULT'){fb.style.display='block';fb.innerHTML='FAULT while in '+esc(S.faultFrom)+': '+esc(S.faultReason||S.reason)}else fb.style.display='none';
  var a=$('armed');a.textContent=S.armed?'ARMED':'DISARMED';a.className='pill '+(S.armed?'on':'off');
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
  if(tab==='servo')renderServo();
  if(tab==='fsr')renderFsr();
  drawTof();
}
$('bArm').onclick=function(){post(S&&S.armed?'/api/fsm/disarm':'/api/fsm/arm')};
$('bOpen').onclick=function(){post('/api/fsm/open')};
$('bReset').onclick=function(){post('/api/fsm/reset')};
$('estop').onclick=function(){post('/api/fsm/estop')};

/* ---------- ToF ---------- */
var maskEdit=[0n,0n,0n],maskLoaded=[null,null,null];
function hex16(m){return BigInt.asUintN(64,m).toString(16).padStart(16,'0')}
function drawTof(){
  var mins=(S&&S.tof&&S.tof.minMm)||[];
  for(var k=0;k<3;k++){
    var sn=T&&T.sensors?T.sensors[k]:null;
    if(tab==='comb'){
      drawHm($('ch'+k),sn,null,false);
      var mm=sn&&sn.online?sn.minMm:(mins[k]===undefined?null:mins[k]);
      $('ch'+k+'t').textContent='S'+(k+1)+(mm===null?' offline':'  min '+mm+' mm');
    }
    if(tab==='tof'){
      drawHm($('th'+k),sn,CFG?maskEdit[k]:null,true);
      $('th'+k+'t').textContent='S'+(k+1)+(sn&&sn.online?'  min '+sn.minMm+' mm':' offline');
    }
  }
  if(tab==='tof'&&T&&T.sensors){
    var h='<tr><th>Sensor</th><th>Online</th><th>ok</th><th>err</th><th>age ms</th></tr>';
    T.sensors.forEach(function(s,k){h+='<tr><td>S'+(k+1)+'</td><td>'+(s.online?'yes':'no')+'</td><td>'+s.ok+'</td><td>'+s.err+'</td><td>'+(s.ageMs===null?'-':s.ageMs)+'</td></tr>'});
    $('tofTbl').innerHTML=h;
  }
}

/* ---------- config forms ---------- */
var FORMS={
 svCfg:[['approachSpeed','Approach speed'],['graspSpeed','Grasp speed'],['openSpeed','Open speed'],['closeTorqueLimit','Close torque limit'],['openTorqueLimit','Open torque limit'],['preTouchTimeoutMs','Pre-touch timeout ms'],['graspTimeoutMs','Grasp timeout ms'],['openTimeoutMs','Open timeout ms'],['openToleranceCounts','Open tolerance counts'],['limitSlackCounts','Limit slack counts'],['runViaPosition','Run via position','b'],['closePositive','Close = positive','b']],
 fsrCfg:[['contactForceN','Contact force N'],['contactSamples','Contact samples'],['graspForceN','Grasp force N'],['graspSamples','Grasp samples'],['maxForceN','Max force N'],['minActiveFsr','Min active FSR per finger']],
 tofCfg:[['detectMm','Detect distance mm'],['detectHoldMs','Detect hold ms']]
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
  var s=S.servo||{},rows=[['Online',s.online?'yes':'NO'],['Age',s.ageMs+' ms'],['Position',s.position],['Speed',s.speed],['Load',fx(s.load,1)],['Current',s.current],['Voltage',fx(s.voltage,1)+' V'],['Temp',s.temp+' C'],['Status',s.status+' ('+esc(s.statusText)+')'],['Mode',s.mode===null?'?':s.mode],['Full angle',s.fullAngle?'yes':'NO'],['Last error',esc(s.lastError||'-')]];
  var h='';rows.forEach(function(r){h+='<tr><th>'+r[0]+'</th><td>'+r[1]+'</td></tr>'});
  $('svTbl').innerHTML=h;
}
var jogTimer=0,jogging=false;
function jogStop(){
  if(!jogTimer&&!jogging)return;
  clearInterval(jogTimer);jogTimer=0;jogging=false;
  post('/api/servo/stop');
}
function jogStart(dir){
  var sp=Math.max(1,Math.min(61,parseInt($('jogSpd').value,10)||20));
  var send=function(){post('/api/servo/jog',{speed:dir*sp})};
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
$('setOpen').onclick=function(){post('/api/servo/setOpen')};
$('setClose').onclick=function(){post('/api/servo/setClose')};
$('scan').onclick=function(){post('/api/servo/scan')};

/* ---------- FSR ---------- */
var CAL=null;
function fingerOf(i){for(var f=0;f<3;f++)if(FMAP[f][0]===i||FMAP[f][1]===i)return f+1;return '-'}
function mkFsr(){
  var h='<tr><th>Ch</th><th>Finger</th><th>raw</th><th>mV</th><th>ohm</th><th>N</th><th>Cal</th><th></th></tr>';
  for(var i=0;i<6;i++)h+='<tr><td>'+FSR_PRINT[i]+'</td><td>'+fingerOf(i)+'</td><td id="fr'+i+'"></td><td id="fm'+i+'"></td><td id="fo'+i+'"></td><td id="ff'+i+'"></td><td id="fc'+i+'"></td><td><button class="b" data-z="'+i+'" style="padding:4px 10px">Zero</button></td></tr>';
  $('fsrTbl').innerHTML=h;
  $('fsrTbl').onclick=function(e){var z=e.target.dataset&&e.target.dataset.z;if(z!==undefined&&z!=='')post('/api/fsr/zero',{id:z}).then(function(j){if(j){toast('Zeroed');loadCal()}})};
}
function renderFsr(){
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
buildForms();mkFingers();mkFsr();loadCfg();
setInterval(pollStatus,200);setInterval(pollTof,200);
pollStatus();pollTof();
</script></body></html>)HTML";
