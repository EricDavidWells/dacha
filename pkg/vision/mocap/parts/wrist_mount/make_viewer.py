"""Write viewer.html: three.js viewer with all STLs embedded. Run after wrist_mount.py."""
import base64, json
import wrist_mount as w

parts = {"base": "wrist_base.stl", "plate_grid": "plate_grid.stl"}
data = {k: base64.b64encode(open(f, "rb").read()).decode() for k, f in parts.items()}
seat_z = w.BASE_T + w.RISE  # plate bottom sits here when assembled

html = """<!doctype html>
<html><head><meta charset="utf-8"><title>Mocap Mount Viewer</title>
<style>
body{margin:0;overflow:hidden;font:13px sans-serif;background:#1e1f22;color:#ddd}
#ui{position:absolute;top:10px;left:10px;background:#000a;padding:10px;border-radius:6px}
#ui label{display:block;margin:4px 0}
</style>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<div id="ui">
<label>Plate <select id="plate"></select></label>
<label><input type="checkbox" id="showBase" checked> Base</label>
<label><input type="checkbox" id="showPlate" checked> Plate</label>
<label><input type="checkbox" id="showWeb" checked> Webbing (approx)</label>
<label>Explode gap <input type="range" id="gap" min="0" max="40" value="15"> <span id="gapv">15</span> mm</label>
<label>Slide (Y) <input type="range" id="slide" min="-60" max="0" value="0"> <span id="slidev">0</span> mm</label>
<div>Drag: rotate. Right drag: pan. Wheel: zoom.</div>
</div>
<script type="module">
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";
import {STLLoader} from "three/addons/loaders/STLLoader.js";
const DATA = __DATA__, SEAT = __SEAT__, W = __WEB__;
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x1e1f22);
const cam = new THREE.PerspectiveCamera(40, innerWidth/innerHeight, 1, 5000);
cam.up.set(0,0,1); cam.position.set(120,-140,110);
const ren = new THREE.WebGLRenderer({antialias:true}); ren.setSize(innerWidth,innerHeight); document.body.appendChild(ren.domElement);
const ctl = new OrbitControls(cam, ren.domElement); ctl.target.set(0,0,15);
scene.add(new THREE.HemisphereLight(0xffffff,0x444444,1.2));
const dl = new THREE.DirectionalLight(0xffffff,1.5); dl.position.set(100,-80,200); scene.add(dl);
const grid = new THREE.GridHelper(200,20,0x555555,0x333333); grid.rotation.x = Math.PI/2; scene.add(grid);
const loader = new STLLoader();
function mesh(key,color){
  const bin = Uint8Array.from(atob(DATA[key]), c=>c.charCodeAt(0)).buffer;
  const g = loader.parse(bin); g.computeVertexNormals();
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({color, metalness:0.1, roughness:0.6}));
  const e = new THREE.LineSegments(new THREE.EdgesGeometry(g,30), new THREE.LineBasicMaterial({color:0x111111}));
  m.add(e); return m;
}
const base = mesh("base",0x4a90d9); scene.add(base);
// webbing: under the base, up through slots, over end bars (approximate path)
const web = new THREE.Group(), wm = new THREE.MeshStandardMaterial({color:0xd8cfb8, roughness:1});
const box=(sx,sy,sz,x,z)=>{const b=new THREE.Mesh(new THREE.BoxGeometry(sx,sy,sz),wm); b.position.set(x,0,z); web.add(b);};
box(W.L+60, W.w, W.t, 0, -W.t/2);
for (const s of [-1,1]){ const xs = s*(W.pad/2+W.slot/2);
  box(W.t, W.w, W.bt+W.t, xs, (W.bt)/2);
  box(W.slot+W.bar, W.w, W.t, s*(W.pad/2+(W.slot+W.bar)/2), W.bt+W.t/2);
  box(W.t, W.w, W.bt+W.t*2, s*(W.L/2+W.t/2), W.bt/2); }
scene.add(web);
const plates = {}; const sel = document.getElementById("plate");
for (const k of Object.keys(DATA)) if (k!=="base"){ plates[k]=mesh(k,0xe08a3c); scene.add(plates[k]); sel.add(new Option(k,k)); }
const $ = id=>document.getElementById(id);
function update(){
  for (const [k,m] of Object.entries(plates)){ m.visible = k===sel.value && $("showPlate").checked;
    m.position.set(0, +$("slide").value, SEAT + +$("gap").value); }
  base.visible = $("showBase").checked; web.visible = $("showWeb").checked;
  $("gapv").textContent=$("gap").value; $("slidev").textContent=$("slide").value;
}
for (const id of ["plate","showBase","showPlate","showWeb","gap","slide"]) $(id).addEventListener("input",update);
update();
addEventListener("resize",()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();ren.setSize(innerWidth,innerHeight);});
(function loop(){requestAnimationFrame(loop);ctl.update();ren.render(scene,cam);})();
</script></body></html>"""

web = {"L": w.BASE_L, "w": w.WEB_W, "t": w.WEB_T, "pad": w.PAD_L, "slot": w.SLOT_L, "bar": w.BAR_L, "bt": w.BASE_T}
html = html.replace("__DATA__", json.dumps(data)).replace("__SEAT__", str(seat_z)).replace("__WEB__", json.dumps(web))
open("viewer.html", "w").write(html)
print("wrote viewer.html")
