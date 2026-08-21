import base64
import json
import os
import re

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
IMG = "docs/img_v6"
DIMS = json.load(open(os.path.join(IMG, "part_dims.json")))


def b64(f, mime="png"):
    return f"data:image/{mime};base64," + base64.b64encode(
        open(os.path.join(IMG, f), "rb").read()).decode()


def M(p, i):
    """alternate text: PROTOTYPE mode / INSERT-BUILD mode."""
    return (f'<span class="mode-proto">{p}</span>'
            f'<span class="mode-ins">{i}</span>')


def strip_tags(s):
    # alt text follows PROTOTYPE mode: drop the insert-build variants
    s = re.sub(r'<span class="mode-ins">.*?</span>', "", s)
    out, depth = [], 0
    for ch in s:
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth -= 1
        elif depth == 0:
            out.append(ch)
    return "".join(out)


def step(num, title, img, fast="", note="", cls=""):
    badge = f'<span class="fast">{fast}</span>' if fast else ""
    note = f'<p class="note">{note}</p>' if note else ""
    alt = strip_tags(title)
    gif = img.replace(".png", ".gif")
    if os.path.exists(os.path.join(IMG, gif)):
        image = (f'<img class="anim" src="{b64(img)}" data-static="{b64(img)}" '
                 f'data-anim="{b64(gif, "gif")}" alt="{alt}">')
        hint = '<span class="hint">&#9658;</span>'
    else:
        image = f'<img src="{b64(img)}" alt="{alt}">'
        hint = ""
    return (f'<div class="step {cls}"><div class="stephead"><span class="num">{num}</span>'
            f'<b>{title}</b>{hint}{badge}</div>{image}{note}</div>')


def part(img, cap):
    return f'<figure><img src="{b64(img)}"><figcaption>{cap}</figcaption></figure>'


def shop(img, title, qty, cap, links):
    a = " &middot; ".join(f'<a href="{u}" target="_blank" rel="noopener">{t} &#8599;</a>'
                          for t, u in links)
    return (f'<figure class="ordcard"><img src="{b64("../img_shop/" + img, "jpeg")}">'
            f'<figcaption><b>{title}</b> <span class="qty">{qty}</span><br>{cap}<br>'
            f'<span class="exlink">example listing: {a}</span></figcaption></figure>')


html = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<title>ATOM shoulder_v6 - assembly</title>
<style>
 body{font-family:'Segoe UI',system-ui,sans-serif;max-width:1240px;margin:0 auto;padding:24px;
      background:#fafbfc;color:#1c2128;line-height:1.5}
 h1{border-bottom:3px solid #EB8C34;padding-bottom:8px;padding-right:180px}
 h2{margin-top:1.6em;border-bottom:1px solid #d8dce2;padding-bottom:4px}
 h2 small{color:#888;font-weight:normal;font-size:60%}
 table{border-collapse:collapse;width:100%;margin:10px 0}
 th,td{border:1px solid #cfd4db;padding:5px 9px;text-align:left;font-size:14px}
 th{background:#eef1f5}
 img{max-width:100%;border-radius:6px;display:block}
 figure{margin:0;background:#fff;border:1px solid #dde;border-radius:8px;padding:6px}
 figcaption{font-size:12.5px;color:#555;margin-top:3px;text-align:center}
 .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:12px 0}
 .g3{grid-template-columns:1fr 1fr 1fr}
 details.pbreak{border:1px solid #c8d0de;border-radius:10px;padding:2px 16px 10px;
   margin:14px 0;background:#eef2f8}
 details.pbreak summary{cursor:pointer;font-size:1.3em;font-weight:700;padding:10px 0}
 details.pbreak summary:hover{color:#0b62c4}
 details.pbreak summary::after{content:" (click to open / close)";font-size:.55em;
   font-weight:400;color:#889}
 .pnote{color:#555;font-size:.92em}
 .ordcard img{max-height:130px;width:auto;max-width:100%;object-fit:contain;
   display:block;margin:0 auto;padding:6px;background:#fff;box-sizing:border-box}
 .ordcard figcaption{text-align:left}
 .qty{background:#EB8C34;color:#fff;border-radius:6px;padding:1px 8px;
   font-weight:700;font-size:.9em;white-space:nowrap}
 .exlink{color:#778;font-size:.85em}
 .exlink a{color:#0b62c4;text-decoration:none}
 .dim{color:#889;font-size:.88em;font-weight:400}
 .steps{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin:12px 0}
 .step{background:#fff;border:1px solid #dde;border-radius:8px;padding:10px}
 .stephead{display:flex;align-items:center;gap:10px;margin-bottom:8px}
 .num{background:#EB8C34;color:#fff;border-radius:14px;min-width:34px;height:28px;
      display:flex;align-items:center;justify-content:center;font-weight:bold;flex:none;
      font-size:13px;padding:0 6px}
 .fast{margin-left:auto;background:#2b3442;color:#e8ecf2;border-radius:14px;
      padding:3px 12px;font-size:13px;white-space:nowrap}
 .hint{font-size:11.5px;color:#EB8C34;border:1px solid #EB8C34;border-radius:10px;
      padding:1px 8px;white-space:nowrap}
 .note{font-size:13px;color:#666;margin:6px 2px 0}
 .crit{background:#fdeaea;border-left:4px solid #C82D2D;padding:10px 14px;margin:12px 0}
 .warn{background:#fff3e6;border-left:4px solid #EB8C34;padding:10px 14px;margin:12px 0}
 .ok{background:#eaf6ec;border-left:4px solid #3a9a4e;padding:10px 14px;margin:12px 0}
 code{background:#eef1f5;padding:1px 5px;border-radius:4px;font-size:13px}
 ul.rules{margin:4px 0 0 0;padding-left:20px}
 ul.rules li{font-size:14px;margin:2px 0}
 #modesel{position:fixed;top:12px;right:12px;z-index:99;cursor:pointer;
   font:600 13px/1.2 system-ui;padding:7px 10px;border-radius:8px;
   border:1px solid #c8ccd6;background:#fff;color:#333}
 body.ins #modesel{border-color:#EB8C34;background:#FBEFE2}
 body.proto .mode-ins,body.proto .ins-only{display:none!important}
 body.ins .mode-proto,body.ins .proto-only{display:none!important}
 @media(max-width:860px){.grid,.steps{grid-template-columns:1fr}}
</style></head><body class="proto">
<select id="modesel" aria-label="build mode">
<option value="proto" selected>without heat inserts</option>
<option value="ins">with heat inserts</option>
</select>

<h1>ATOM shoulder_v6 &mdash; assembly</h1>
<p><b>Hover any card to watch it happen.</b> Right module:
<code>cad/shoulder_v6/right/</code>.</p>
__HERO__
<div class="ok"><b>Verified:</b> every path swept clear
(<code>sim36/interference_check.py</code>).</div>

<h2>Before you build</h2>
<p class="mode-proto"><b>No thread prep.</b> Every M3 hole is printed into the
part &mdash; drive the screw, it makes its own thread.</p>
<div class="steps">
__PREP1__
__PREP2__
<figure style="grid-column:1/-1"><img src="__LEGENDIMG__" alt="fastener legend"
style="max-width:720px;width:100%"><figcaption>shape + shade = type; dot = driven
head<span class="mode-ins">; brass = insert</span>.</figcaption></figure>
</div>
<div class="crit"><b>Ordering rules:</b>
<ul class="rules">
<li class="mode-ins">All 30 inserts before assembly.</li>
<li>All servos to <code>2048</code> before any horn.</li>
<li>Bench-fit the pitch + roll horns: B4, C1. Twist horn goes on at C3.</li>
<li>D1 before C5; C4 before the plate.</li>
</ul></div>

<details class="pbreak">
<summary>Part breakdown &mdash; print list &amp; order list</summary>
<h3>Print &mdash; one set per arm</h3>
<p class="pnote">Right arm: mirrored set in <code>cad/shoulder_v6/right/</code>.
<span class="mode-proto">All M3 threads are printed into the parts.</span><br>
__PRINTSIZE__</p>
<div class="grid g3">
__PRINTS__
</div>
<h3>Order</h3>
<div class="grid">
__ORDERS__
</div>
<p class="pnote"><span class="mode-ins">Also order: 30&times; M3 heat-set inserts
(4.0&nbsp;mm short, one SKU) + 4&times; M4 for the bicep. </span>Loctite 641
(pitch seat, twist race) &middot; grease (M5 shank).</p>
</details>

<div class="mode-ins">
<h2>P &mdash; heat-set inserts</h2>
<p>Every insert while the part is bare.</p>
<div class="steps">
__P1__
__P2__
__P3__
__P4__
</div>
</div>

<h2>A &mdash; pitch base</h2>
<div class="steps">
__A1__
__A2__
</div>

<h2>B &mdash; carrier</h2>
<div class="steps">
__B1__
__B2__
__B3__
__B4__
</div>

<h2>C &mdash; yoke + twist stack</h2>
<div class="steps">
__C1__
__C2__
__C3__
__C4__
__D1__
__C5__
__C6__
__C7__
__C8__
</div>

<h2>D &mdash; bicep</h2>
<div class="steps">
<div class="step"><div class="stephead"><span class="num">D2</span>
<b>confirm all three servos still read 2048 before F</b></div></div>
</div>

<h2>F &mdash; final assembly</h2>
<div class="steps">
__F1__
__F2__
__F3__
__F4__
</div>

<h2>Checks</h2>
<table>
<tr><th>test</th><th>pass</th></tr>
<tr><td>sweep each axis, unpowered</td><td>stops only at limits: pitch &minus;60..180&deg;,
roll &minus;25..150&deg;, twist &plusmn;90&deg;</td></tr>
<tr><td>pull down on the bicep tray</td><td>no play (else M3&times;20 loose)</td></tr>
<tr><td>twist a module on the spine</td><td>no rotation; shoulders level</td></tr>
<tr><td>power all three at 2048</td><td>hangs straight, tray square</td></tr>
<tr><td>twist cabling</td><td>service loop &mdash; no slip ring</td></tr>
</table>
<div class="warn">Bearings are sealed for life: no grease.</div>

<p style="margin-top:2em;color:#777;font-size:12.5px"><code>build_shoulder_v6_cq.py</code>
&middot; <code>sim36/interference_check.py</code></p>
<script>
document.querySelectorAll("img.anim").forEach(function(im){
  im.addEventListener("mouseenter",function(){im.src=im.dataset.anim;});
  im.addEventListener("mouseleave",function(){im.src=im.dataset.static;});
});
document.getElementById("modesel").addEventListener("change",function(){
  var ins=this.value==="ins",b=document.body;
  b.classList.toggle("ins",ins);b.classList.toggle("proto",!ins);
});
</script>
</body></html>"""

S = {
 "__HERO__": ('<figure class="hero"><img src="' + b64("hero.gif", "gif")
              + '" alt="shoulder_v6 sweeping pitch, roll and twist">'
              '<figcaption>shoulder_v6 &mdash; pitch &middot; roll &middot; twist</figcaption></figure>'),
 "__PRINTS__": "\n".join(
    part("part_%s.png" % n,
         '<code>%s.stl</code> <span class="dim">%d&times;%d&times;%d mm</span>'
         % ((n,) + tuple(round(v) for v in DIMS[n]))) for n in [
        "mount", "carrier", "yoke", "hub_clamp",
        "interface_plate", "pitch_retainer", "race_cap"]),
 "__PRINTSIZE__": (lambda big=max(
        ["mount", "carrier", "yoke", "hub_clamp", "interface_plate",
         "pitch_retainer", "race_cap"], key=lambda n: max(DIMS[n])):
     "largest part: <code>%s.stl</code> at %d&times;%d&times;%d mm &mdash; "
     "everything fits a 150&times;150&times;150 mm bed. "
     "Assembled module: 147 mm tall, shoulder axis to bicep face 102 mm."
     % ((big,) + tuple(round(v) for v in DIMS[big])))(),
 "__ORDERS__": "\n".join([
    shop("hts35h_ref_1.jpg", "Servos", "3&times; per arm",
         "Hiwonder HTS-35H serial-bus, 35 kg&middot;cm @ 11.1 V. Label them P, R, T.",
         [("HTS-35H", "https://www.amazon.com/Hiwonder-Channels-Temperature-Position-Feedback/dp/B0C9CZXWXR")]),
    shop("servohorn_1.jpg", "25T round disc horns", "3&times; per arm",
         "Aluminium <b>ROUND DISC</b> &mdash; &Oslash;20 disc, 4&times; M3 on &Oslash;14. "
         "NOT arm/X &ldquo;steering&rdquo; horns. Drill the pitch + roll discs to "
         "&Oslash;3.2; the twist disc stays factory-tapped.",
         [("Honbay 6-pack", "https://www.amazon.com/Honbay-Aluminum-Standard-Airplane-Hop-up/dp/B09CT7QK6C")]),
    shop("bearings_1.jpg", "Bearings &mdash; 3 different kinds", "1&times; each per arm",
         "6808-2RS (40&times;52&times;7) pitch &middot; 6806-2RS (30&times;42&times;7) "
         "twist &middot; 625ZZ (5&times;16&times;5) roll idler",
         [("6808", "https://www.amazon.com/dp/B0CRLGD5GL"),
          ("6806", "https://www.amazon.com/dp/B0DJ6LY17T"),
          ("625ZZ", "https://www.amazon.com/dp/B0C9J3V988")]),
    shop("fasteners_1.jpg", "Fasteners", "per arm",
         "12&times; M3&times;8 &middot; 12&times; M3&times;6 (+1 factory centre) &middot; "
         "3&times; M3&times;8 button &middot; 4&times; M3&times;8 CSK &middot; "
         "3&times; M3&times;20 &middot; 4&times; M4&times;16 &middot; "
         "4&times; M3&times;80 + nyloc &middot; M5&times;25 + nyloc + &Oslash;5&times;4 spacer",
         [("M3 kit", "https://www.amazon.com/dp/B0GT8W6JVZ"),
          ("M3&times;80", "https://www.amazon.com/dp/B0DNSKT2TS")]),
 ]),
 
 
 "__PREP1__": step("i", "threads: brass heat-set inserts", "inserts.png",
                   "30&times; M3 insert",
                   "One SKU, ironed flush into a &Oslash;4.2 bore.",
                   cls="mode-ins"),
 "__LEGENDIMG__": b64("legend.png"),
 "__PREP2__": step("ii", "how a servo drives a joint", "drive.png",
                   "spline &rarr; horn &rarr; part",
                   "Horn grips the spline, bolts to the part."),

 "__P1__": step("P1", "mount: 4 face-plate + 3 retainer-boss inserts",
                "p1.png", "7&times; M3 insert",
                "the boss three are unreachable after A1"),
 "__P2__": step("P2", "carrier: 4 face-plate + 4 pitch-horn inserts",
                "p2.png", "8&times; M3 insert"),
 "__P3__": step("P3", "yoke: 4 race-cap + 4 face-plate + 4 roll-horn inserts",
                "p3.png", "12&times; M3 insert"),
 "__P4__": step("P4", "hub clamp: 3 top-face inserts",
                "p4.png", "3&times; M3 insert"),

 "__A1__": step("A1", "press the 6808 into the boss seat", "a1.png"),
 "__A2__": step("A2", "cap it with the pitch retainer", "a2.png",
                "3&times; M3&times;8 button head",
                "button heads clear the pitch journal"),

 "__B1__": step("B1", "servo R slides in - with its four screws already dropped in the case",
                "b1.png"),
 "__B2__": step("B2", "four ear screws up from the open side", "b2.png",
                "4&times; M3&times;8 socket cap"),
 "__B3__": step("B3", "press the 625ZZ into the idler seat", "b3.png"),
 "__B4__": step("B4", "pitch horn onto the journal end - on the bench, NOW", "b4.png",
                "4&times; M3&times;6 socket cap"),

 "__C1__": step("C1", "roll horn onto the yoke boss - on the bench", "c1.png",
                "4&times; M3&times;6 socket cap",
                "servo R&rsquo;s spline drives into THIS horn at F3"),
 "__C2__": step("C2", "servo T rises, ears onto the deck", "c2.png",
                "4&times; M3&times;8 socket cap"),
 "__C3__": step("C3", "twist horn (round disc) down the open bore, onto the live spline", "c3.png",
                "1&times; factory M3&times;6 centre screw",
                "the only moment this screw is reachable"),
 "__C4__": step("C4", "hub clamp down onto the horn", "c4.png",
                "4&times; M3&times;6 socket cap",
                "into the horn&rsquo;s own tapped holes"),
 "__D1__": step("D1", "bicep to the plate - before the plate is installed", "d1.png",
                "4&times; M4&times;16 socket cap"),
 "__C5__": step("C5", "6806 into the platform seat", "c5.png"),
 "__C6__": step("C6", "race cap + screws - BEFORE the plate", "c6.png",
                "4&times; M3&times;8 countersunk",
                "the plate sits 1.5 mm over these heads"),
 "__C7__": step("C7", "plate + bicep assembly: journal down through the capped bearing",
                "c7.png"),
 "__C8__": step("C8", "hub bolts down the journal, through the bicep cavity", "c8.png",
                "3&times; M3&times;20 socket cap",
                "these three carry the hanging arm"),

 "__F1__": step("F1", "carrier assembly (horn already fitted) into the mount", "f1.png", "",
                "Loctite 641 on the journal seat"),
 "__F2__": step("F2", "servo P (clocked 90&deg;) up the channel, spline into the horn", "f2.png",
                "4&times; M3&times;8 socket cap", "servo at 2048"),
 "__F3__": step("F3", "yoke assembly onto the arm - servo R&rsquo;s spline runs up into the C1 horn", "f3.png",
                "1&times; M5&times;25 + M5 nyloc + &Oslash;5&times;4 spacer",
                "grease the M5 shank"),
 "__F4__": step("F4", "spine sandwich: push the four M3&times;80 through the shell as STUDS, "
                "slide the spine on, far shell, nylocs", "f4.png",
                "4&times; M3&times;80 socket cap + M3 nyloc nuts"),
}
for k, v in S.items():
    html = html.replace(k, v)
open("docs/shoulder_v6_assembly.html", "w", encoding="utf-8").write(html)
print("wrote docs/shoulder_v6_assembly.html  (%.1f MB)"
      % (os.path.getsize("docs/shoulder_v6_assembly.html") / 1e6))
