# Dimension Sheet

Draw CAD-style dimension lines on photos, type in the measurements, export a PDF
drawing sheet. Built for documenting the rover chassis, but it takes any image.

## Running it

Double-click `index.html`. That's the whole install.

It works from `file://` with no server and no internet — jsPDF is vendored in
`vendor/`, and every image you load is read locally through the File API and
never leaves the machine. The one network request is the Google Fonts
stylesheet; offline it just falls back to the system font stack and everything
else behaves identically.

Chrome, Firefox and Safari are all fine. No build step, no dependencies to
install.

## Using it

| Key | Tool |
|-----|------|
| `V` | Select — click a dimension, drag its endpoints or the middle handle |
| `D` | Dimension — drag across a feature |
| `L` | Callout — arrow plus free text, for labelling parts |
| `C` | Calibrate — drag a known length, enter its real size |

`Shift` while dragging locks to horizontal, vertical or 45°. Arrow keys nudge
the selection (`Shift` for 10 px steps). `Delete` removes it. `Ctrl+Z` undoes.

**The normal workflow:** drop a photo in, press `C` and drag across something
whose real size you know, type that number — then every dimension you draw
afterwards reads out in real units automatically. If you'd rather not calibrate,
just draw dimensions and type each value into the Selected panel by hand.

**Save** writes a `.json` project holding the photos and all annotations, so you
can reopen and keep editing later. **Open** loads one back.

## Choosing a calibration reference

Error scales inversely with reference length, so calibrate on the *longest*
known feature lying in the *same plane* as whatever you're measuring:

- **Best:** a frame rail you measured with a tape. Long, straight, and no
  guessing at catalogue numbers.
- **Good:** wheel diameter. A circle seen at an angle projects to an ellipse
  whose *major axis* is unforeshortened, so measure across the widest part of
  the tyre and you get the true diameter even on a tilted wheel. Caliper the
  real thing — nominal RC tyre sizes vary by several mm between brands.
- **Avoid:** PVC outer diameter. It spans ~30 px in a typical photo, so a 2 px
  click error becomes ~7% on every derived dimension. Nominal PVC sizes aren't
  actual ODs either.

Calibration is per-view — each photo has its own scale, which is correct, since
different shots are at different distances.

## The caveat that matters

These are perspective photographs, not orthographic projections. A scale
calibrated in one plane is only valid in that plane; anything measured across
depth will read short. For dimensions that have to be right, measure the rover
itself and type the number in. The tool is doing the drawing, not the metrology.

## How it works

One file, no framework. Annotations are stored in **image pixel coordinates**
and the canvas transform is set to `dispScale * dpr` before drawing, so the same
code paints the on-screen preview and the full-resolution export — export just
resets the scale to 1 and hides the drag handles. Nothing is ever baked into the
JPEG; the photo is redrawn from scratch every frame with the annotations on top,
which is why you can drag endpoints and undo indefinitely without degrading it.

A dimension is derived from two dragged points:

```js
function dimEnds(an){
  var d = norm(sub(an.p2, an.p1)), n = perp(d);   // direction + normal
  return { d:d, n:n, a1:add(an.p1,n,an.off), a2:add(an.p2,n,an.off) };
}
```

The offset slides the dimension line along the normal `n`, extension lines run
from the original points out past it, arrowheads are filled triangles oriented
along `d`, and the label chip is rotated to `atan2(d.y, d.x)` — flipped by π
when that would leave the text upside down.

Images are deliberately loaded as `data:` URLs rather than by path. A
`file://`-loaded `<img src="photo.jpg">` taints the canvas and makes
`toBlob`/`toDataURL` throw, which would break both exports; `data:` URLs don't
taint, so export works even when the page is opened straight off the disk.

There's a hosted copy at <https://claude.ai/artifact/PMqdyFwEJ9jAr8wce5ZfKr>
with the two rover photos preloaded. It's the same code, differing only in how
it hands you the finished file.
