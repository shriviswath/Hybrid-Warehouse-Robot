// Phone UI for the hybrid robot. Served from flash, no filesystem needed.
#pragma once
#include <pgmspace.h>

const char MANIFEST_JSON[] PROGMEM = R"J({
"name":"HybridBot","short_name":"HybridBot","start_url":"/",
"display":"fullscreen","orientation":"any","background_color":"#0b0f14",
"theme_color":"#0b0f14","icons":[]
})J";

const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#0b0f14">
<link rel="manifest" href="/manifest.json">
<title>HybridBot</title>
<style>
*{box-sizing:border-box;margin:0;-webkit-tap-highlight-color:transparent}
html,body{height:100%;overflow:hidden}
body{background:#0b0f14;color:#e6edf3;font:13px/1.35 -apple-system,system-ui,"Segoe UI",Roboto,sans-serif;
 padding:8px env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left);
 display:flex;flex-direction:column;gap:8px;touch-action:none;user-select:none}
.row{display:flex;gap:8px;align-items:center}
.card{background:#131a22;border:1px solid #1f2a36;border-radius:12px;padding:9px}
.tabs{display:flex;background:#131a22;border:1px solid #1f2a36;border-radius:12px;padding:3px;flex:1}
.tabs button{flex:1;background:none;border:0;color:#7d8da0;font:700 13px/1 inherit;padding:10px;border-radius:9px}
.tabs button.on{background:#1d84ff;color:#fff}
.tabs button.on.warn{background:#d97706}
#estop{width:86px;height:44px;border-radius:12px;border:1px solid #7f1d1d;background:#2b1113;
 color:#fda4a4;font:800 12px/1 inherit;letter-spacing:.5px}
#estop.latched{background:#dc2626;color:#fff;border-color:#dc2626}
.tel{display:grid;grid-template-columns:repeat(5,1fr);gap:5px;font-variant-numeric:tabular-nums}
.tel div{background:#0f1620;border-radius:8px;padding:5px 6px;min-width:0}
.tel span{display:block;color:#64748b;font-size:9px;text-transform:uppercase;letter-spacing:.5px}
.tel b{font-size:13px;font-weight:700;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;display:block}
.sticks{flex:1;display:flex;gap:8px;min-height:150px}
.pad{position:relative;flex:1;background:radial-gradient(circle at 50% 50%,#17212c 0%,#101722 72%);
 border:1px solid #1f2a36;border-radius:16px;overflow:hidden}
.pad:before,.pad:after{content:"";position:absolute;background:#1f2a36}
.pad:before{left:0;right:0;top:50%;height:1px}
.pad:after{top:0;bottom:0;left:50%;width:1px}
.knob{position:absolute;width:66px;height:66px;margin:-33px 0 0 -33px;left:50%;top:50%;border-radius:50%;
 background:#1d84ff;box-shadow:0 5px 16px rgba(29,132,255,.45);transition:transform .08s}
.pad.live .knob{transition:none}
#padL .knob{background:#16a34a;box-shadow:0 5px 16px rgba(22,163,74,.45)}
.lab{position:absolute;left:8px;top:6px;color:#475569;font-size:10px;letter-spacing:.5px}
.lab2{position:absolute;right:8px;bottom:6px;color:#475569;font-size:10px}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.grid2{display:grid;grid-template-columns:repeat(2,1fr);gap:6px}
button.b{padding:12px 0;border-radius:10px;border:1px solid #243040;background:#182230;color:#e6edf3;
 font:700 12px/1 inherit}
button.b:active{background:#243040}
button.b.sel{background:#1d84ff;border-color:#1d84ff;color:#fff}
button.b.red{background:#2b1113;border-color:#7f1d1d;color:#fda4a4}
button.b.grn{background:#07291a;border-color:#166534;color:#86efac}
label{display:block;color:#7d8da0;font-size:10px;text-transform:uppercase;letter-spacing:.5px;margin-bottom:4px}
label b{color:#e6edf3;float:right;font-variant-numeric:tabular-nums}
input[type=range]{width:100%;height:30px;-webkit-appearance:none;background:none}
input[type=range]::-webkit-slider-runnable-track{height:7px;border-radius:4px;background:#243040}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:28px;height:28px;margin-top:-10px;
 border-radius:50%;background:#e6edf3;border:0}
.tune{margin-top:8px}
.tune summary{list-style:none;cursor:pointer;color:#7d8da0;font-size:10px;
 text-transform:uppercase;letter-spacing:.6px;padding:6px 0}
.tune summary::-webkit-details-marker{display:none}
.tune summary:after{content:" ▸";color:#475569}
.tune[open] summary:after{content:" ▾"}
.tworow{display:flex;gap:8px;align-items:center;margin-bottom:4px}
.tworow span{color:#64748b;font-size:10px;min-width:72px;text-transform:uppercase;letter-spacing:.5px}
.tworow input[type=range]{flex:1;height:26px}
.tworow b{font-variant-numeric:tabular-nums;min-width:34px;text-align:right;font-weight:700}
.duty{font-variant-numeric:tabular-nums;color:#7d8da0;font-size:11px;margin-top:4px}
.armhdr{display:flex;justify-content:space-between;color:#64748b;font-size:10px;
 text-transform:uppercase;letter-spacing:.6px;margin-bottom:6px}
.armhdr span{color:#86efac;font-weight:700}
#armcard.locked{opacity:.55;pointer-events:none}
#armcard.locked .armhdr span{color:#7d8da0;font-weight:400;text-transform:none}
.hide{display:none!important}
#link{width:9px;height:9px;border-radius:50%;background:#dc2626;flex:0 0 auto}
#link.up{background:#16a34a}
.mut{color:#64748b;font-size:10px}
.warnbar{background:#2b1a07;border:1px solid #92400e;color:#fcd34d;border-radius:9px;padding:6px 8px;font-size:11px}
</style></head><body>

<div class="row">
  <div class="tabs">
    <button id="tg" class="on">GROUND</button>
    <button id="tf" class="warn">FLIGHT</button>
  </div>
  <button id="estop" class="latched">E-STOP</button>
</div>

<div class="card tel">
  <div><span>State</span><b id="vst">--</b></div>
  <div><span>Mode</span><b id="vmode">--</b></div>
  <div><span>Batt</span><b id="vbat">--</b></div>
  <div><span>Alt</span><b id="valt">--</b></div>
  <div><span>Climb</span><b id="vcl">--</b></div>
</div>

<!-- ============ FLIGHT ============ -->
<div id="fly" class="hide" style="flex:1;display:flex;flex-direction:column;gap:8px">
  <div id="stabwarn" class="warnbar hide">
    STABILIZE: throttle does NOT self-centre. Releasing the stick holds the
    last value. Switch back to ALT_HOLD unless you know why you are here.
  </div>
  <div class="sticks" style="min-height:130px">
    <div class="pad" id="padL"><div class="knob"></div>
      <div class="lab">THROTTLE / YAW</div><div class="lab2" id="thrv">--</div></div>
    <div class="pad" id="padR"><div class="knob"></div>
      <div class="lab">PITCH / ROLL</div></div>
  </div>
  <div class="grid4">
    <button class="b" data-c="alt">ALT HOLD</button>
    <button class="b" data-c="loi">LOITER</button>
    <button class="b" data-c="rtl">RTL</button>
    <button class="b" data-c="land">LAND</button>
  </div>
  <div class="grid2">
    <button class="b grn" id="barm">ARM</button>
    <button class="b red" id="bdis">DISARM</button>
  </div>
  <div class="grid2">
    <button class="b" id="bstab">STABILIZE</button>
    <button class="b" id="bthrmin">THROTTLE MIN</button>
  </div>
</div>

<!-- ============ GROUND ============ -->
<div id="gnd" style="flex:1;display:flex;flex-direction:column;gap:8px">
  <div class="sticks"><div class="pad" id="padG"><div class="knob"></div>
    <div class="lab">THROTTLE / STEER</div><div class="lab2" id="dutyv">0 / 0</div></div></div>
  <details class="tune card">
    <summary>Drive tuning</summary>
    <div class="tworow"><span>Max duty</span>
      <input id="tmax" type="range" min="60" max="255" value="255"><b id="vmax">255</b></div>
    <div class="tworow"><span>Floor</span>
      <input id="tfloor" type="range" min="0" max="180" value="60"><b id="vfloor">60</b></div>
    <div class="tworow"><span>Deadband</span>
      <input id="tdead" type="range" min="0" max="40" value="8"><b id="vdead">8</b></div>
    <div class="grid4" style="margin-top:6px">
      <button class="b" data-q="500">500 Hz</button>
      <button class="b" data-q="1000">1 kHz</button>
      <button class="b" data-q="2000">2 kHz</button>
      <button class="b" data-q="5000">5 kHz</button>
    </div>
    <div class="grid2" style="margin-top:6px">
      <button class="b" id="binvl">INVERT LEFT</button>
      <button class="b" id="binvr">INVERT RIGHT</button>
    </div>
    <div class="duty" id="tunehint">Pack too low? raise Max duty. Motors buzz but
      do not turn? raise Floor. L298N likes 500-1000 Hz.</div>
  </details>
</div>

<!-- ======= MANIPULATOR: common to both modes ======= -->
<div id="armwrap">
  <div class="card" id="armcard">
    <div class="armhdr">MANIPULATOR<span id="armstate">live</span></div>
    <label>Rack <b id="vrack">180</b></label>
    <input id="rack" type="range" min="40" max="180" value="180">
    <label style="margin-top:2px">Gripper <b id="vgrip">60</b></label>
    <input id="grip" type="range" min="0" max="60" value="60">
    <div class="grid4" style="margin-top:8px">
      <button class="b" data-g="ext">EXTEND</button>
      <button class="b" data-g="ret">RETRACT</button>
      <button class="b" data-g="op">OPEN</button>
      <button class="b" data-g="cl">CLOSE</button>
    </div>
  </div>
</div>

<div class="row"><div id="link"></div><div class="mut" id="lmsg">connecting...</div></div>

<script>
var ws, mode=0, estop=1, armed=0, apMode='', apLive=0, alive=0;
var E=function(i){return document.getElementById(i)};
var invL=0, invR=0, freqHz=1000, tuneTouched=false;
var F={roll:0,pitch:0,yaw:0,thr:-100};     // flight sticks
var G={thr:0,steer:0};                      // ground

function send(s){ if(ws&&ws.readyState===1) ws.send(s); }
function selfCentreThrottle(){ return apMode!=='STABILIZE'; }

/* ---------------- websocket ---------------- */
function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){ E('link').className='up'; E('lmsg').textContent='linked';
    send('E,'+estop); send('M,'+mode); };
  ws.onclose=function(){ E('link').className=''; E('lmsg').textContent='link lost - retrying';
    setTimeout(connect,700); };
  ws.onmessage=function(e){
    var p=e.data.split(','); if(p[0]!=='T') return;
    alive=Date.now();
    mode=+p[1]; estop=+p[2]; armed=+p[3]; apLive=+p[4]; apMode=p[5];
    E('vst').textContent   = estop? 'E-STOP' : (armed? 'ARMED':'disarmed');
    E('vst').style.color   = estop? '#fda4a4' : (armed? '#fcd34d':'#86efac');
    E('vmode').textContent = mode? (apLive? apMode : 'NO FC') : 'GROUND';
    E('vbat').textContent  = (+p[6]).toFixed(1)+'V';
    E('valt').textContent  = (+p[8]).toFixed(1)+'m';
    E('vcl').textContent   = (+p[9]).toFixed(1);
    E('estop').className   = estop? 'latched':'';
    E('tg').className = mode? '' : 'on';
    E('tf').className = mode? 'on warn' : 'warn';
    E('fly').classList.toggle('hide', mode===0);
    E('gnd').classList.toggle('hide', mode===1);
    E('stabwarn').classList.toggle('hide', !(apLive && apMode==='STABILIZE'));
    var locked = +p[15];
    E('armcard').classList.toggle('locked', !!locked);
    E('armstate').textContent = !locked ? 'live'
        : estop ? 'holding - E-STOP latched'
                : 'holding - switch to GROUND to move';
    E('dutyv').textContent = p[12]+' / '+p[13];
    if(!tuneTouched){
      E('tmax').value=p[18];   E('vmax').textContent=p[18];
      E('tfloor').value=p[19]; E('vfloor').textContent=p[19];
      E('tdead').value=p[20];  E('vdead').textContent=p[20];
      invL=+p[21]; invR=+p[22]; freqHz=+p[23];
      E('binvl').className='b'+(invL?' sel':'');
      E('binvr').className='b'+(invR?' sel':'');
      document.querySelectorAll('[data-q]').forEach(function(b){
        b.className='b'+(+b.getAttribute('data-q')===freqHz?' sel':'');
      });
    }
    if(locked){                       // mirror the real angles the servos hold
      E('rack').value=p[16]; E('grip').value=p[17];
      E('vrack').textContent=p[16]; E('vgrip').textContent=p[17];
    }
    document.querySelectorAll('[data-c]').forEach(function(b){
      var m={alt:'ALT_HOLD',loi:'LOITER',rtl:'RTL',land:'LAND'}[b.getAttribute('data-c')];
      b.className = 'b' + (apMode===m ? ' sel' : '');
    });
  };
}
connect();

/* -------------- 20 Hz command stream -------------- */
setInterval(function(){
  if(mode===1) send('F,'+Math.round(F.roll)+','+Math.round(F.pitch)+','+
                        Math.round(F.yaw)+','+Math.round(F.thr));
  else         send('D,'+Math.round(G.thr)+','+Math.round(G.steer));
},50);

/* -------------- stick plumbing -------------- */
function makePad(id, onMove, onRelease, springX, springY){
  var pad=E(id), knob=pad.querySelector('.knob'), active=null;
  function place(x,y){
    var r=pad.getBoundingClientRect();
    knob.style.transform='translate('+(x*(r.width/2-36))+'px,'+(y*(r.height/2-36))+'px)';
  }
  function at(e){
    var r=pad.getBoundingClientRect();
    var x=Math.max(-1,Math.min(1,(e.clientX-r.left)/r.width*2-1));
    var y=Math.max(-1,Math.min(1,(e.clientY-r.top)/r.height*2-1));
    onMove(x,y); place(x,y);
  }
  function rel(){
    if(active===null) return;
    active=null; pad.classList.remove('live');
    var s=onRelease(); place(springX?0:s.x, springY?0:s.y);
  }
  pad.addEventListener('pointerdown',function(e){ active=e.pointerId;
    pad.setPointerCapture(e.pointerId); pad.classList.add('live'); at(e); e.preventDefault(); });
  pad.addEventListener('pointermove',function(e){ if(e.pointerId===active) at(e); });
  pad.addEventListener('pointerup',rel);
  pad.addEventListener('pointercancel',rel);
  pad.addEventListener('lostpointercapture',rel);
  return {place:place, release:rel};
}

/* LEFT stick: X = yaw (always springs), Y = throttle.
   Throttle springs to centre in ALT_HOLD / LOITER, where centre means HOLD
   ALTITUDE. In STABILIZE it must NOT spring -- releasing would cut power. */
var padL = makePad('padL',
  function(x,y){ F.yaw=x*100; F.thr=-y*100; E('thrv').textContent=Math.round(F.thr); },
  function(){
    F.yaw=0;
    if(selfCentreThrottle()){ F.thr=0; E('thrv').textContent='0 (hold)'; return {x:0,y:0}; }
    return {x:0, y:-F.thr/100};
  }, true, false);

/* RIGHT stick: pitch / roll, both always spring to centre. */
var padR = makePad('padR',
  function(x,y){ F.roll=x*100; F.pitch=-y*100; },
  function(){ F.roll=0; F.pitch=0; return {x:0,y:0}; }, true, true);

/* GROUND pad */
var padG = makePad('padG',
  function(x,y){ G.steer=x*100; G.thr=-y*100; },
  function(){ G.thr=0; G.steer=0; send('D,0,0'); return {x:0,y:0}; }, true, true);

/* put the throttle at minimum on load and whenever we enter flight */
function throttleMin(){ F.thr=-100; E('thrv').textContent='-100 (min)';
  padL.place(0,1); }
throttleMin();

/* -------------- buttons -------------- */
E('tg').onclick=function(){ send('M,0'); };
E('tf').onclick=function(){
  if(confirm('FLIGHT MODE\n\nWheels stop, arm locks, throttle resets to minimum.\nProps clear?'))
  { throttleMin(); F.roll=F.pitch=F.yaw=0; padR.place(0,0); send('M,1'); }
};
E('estop').onclick=function(){
  if(estop===0){ estop=1; send('E,1'); return; }         // latching never asks
  if(confirm('Clear E-STOP? Motors become live.')){ estop=0; send('E,0'); }
};
E('barm').onclick=function(){
  if(Math.round(F.thr)>-95){ alert('Throttle is not at minimum.\nPull it all the way down first.'); return; }
  if(confirm('ARM the autopilot?\n\nProps will spin. Everyone clear?')) send('A,1');
};
E('bdis').onclick=function(){ send('A,0'); };
E('bthrmin').onclick=throttleMin;
E('bstab').onclick=function(){
  if(confirm('STABILIZE?\n\nThrottle will NOT self-centre. Releasing the stick holds\n'+
             'the last value, and there is no altitude hold.\n\nAre you sure?'))
    send('C,stab');
};
document.querySelectorAll('[data-c]').forEach(function(b){
  b.onclick=function(){ send('C,'+b.getAttribute('data-c')); };
});

/* -------------- ground arm controls -------------- */
function servos(){
  E('vrack').textContent=E('rack').value;
  E('vgrip').textContent=E('grip').value;
  send('S,'+E('rack').value+','+E('grip').value);
}
E('rack').addEventListener('input',servos);
E('grip').addEventListener('input',servos);
document.querySelectorAll('[data-g]').forEach(function(b){
  b.onclick=function(){
    var g=b.getAttribute('data-g');
    if(g==='ext') E('rack').value=40;
    if(g==='ret') E('rack').value=180;
    if(g==='op')  E('grip').value=60;
    if(g==='cl')  E('grip').value=0;
    servos();
  };
});

/* -------------- drive tuning -------------- */
function sendTune(){
  tuneTouched=true;
  E('vmax').textContent=E('tmax').value;
  E('vfloor').textContent=E('tfloor').value;
  E('vdead').textContent=E('tdead').value;
  send('G,'+E('tmax').value+','+E('tfloor').value+','+E('tdead').value);
}
['tmax','tfloor','tdead'].forEach(function(id){
  E(id).addEventListener('input',sendTune);
  E(id).addEventListener('change',function(){ setTimeout(function(){tuneTouched=false;},1500); });
});
E('binvl').onclick=function(){ invL=invL?0:1; send('I,'+invL+','+invR);
  E('binvl').className='b'+(invL?' sel':''); };
E('binvr').onclick=function(){ invR=invR?0:1; send('I,'+invL+','+invR);
  E('binvr').className='b'+(invR?' sel':''); };
document.querySelectorAll('[data-q]').forEach(function(b){
  b.onclick=function(){
    freqHz=+b.getAttribute('data-q'); send('Q,'+freqHz);
    document.querySelectorAll('[data-q]').forEach(function(x){
      x.className='b'+(+x.getAttribute('data-q')===freqHz?' sel':''); });
  };
});

/* -------------- safety nets -------------- */
function panicRelease(){
  F.roll=F.pitch=F.yaw=0; padR.place(0,0);
  G.thr=0; G.steer=0;
  if(selfCentreThrottle()){ F.thr=0; padL.place(0,0); }
}
window.addEventListener('blur',panicRelease);
document.addEventListener('visibilitychange',function(){ if(document.hidden) panicRelease(); });
setInterval(function(){
  if(alive && Date.now()-alive>1200) E('lmsg').textContent='no telemetry';
},500);
</script></body></html>
)HTML";
