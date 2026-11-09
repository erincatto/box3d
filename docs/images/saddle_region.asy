settings.outformat = "svg";
settings.render = 0;
settings.prc = false;

import three;

real px = 0.75bp;

pen ink = rgb("231f20");
pen meshPen = ink + 0.6px;
pen regionFill = rgb("f2d4d0");
pen regionPen = rgb("a8322a") + 1.0px;
pen ridgeColor = rgb("c26a17");
pen gullyColor = rgb("1f5c99");
pen sliceFill = gray(0.93) + opacity(0.75);
pen guidePen = gray(0.55) + 0.5px + linetype(new real[] {4, 3});
pen hullPen = ink + 0.7px + linetype(new real[] {5, 3});

real arrowSize = 5px;
triple lightDirection = unit((1, 4, 2));

// The Saddle Vertex sample hub, spokes alternate 8 degree ridges and 32 degree gullies
int spokeCount = 6;
real fanRadius = 1.5;
real[] drops = {8, 32, 8, 32, 8, 32};

// The slice plane sits this far above the hub. Planes through the hub cut it in lines.
real sliceHeight = 1.6;
real sliceRadius = sliceHeight * Tan(40);

triple[] spokes;
for (int i = 0; i < spokeCount; ++i)
{
  real angle = 60 * i;
  spokes.push(fanRadius * (Cos(angle), -Tan(drops[i]), Sin(angle)));
}

bool IsRidge(int i)
{
  return i % 2 == 0;
}

pen SpokePen(int i)
{
  return IsRidge(i) ? ridgeColor : gullyColor;
}

// Gnomonic coordinates: a normal n maps to (n.x, n.z) / n.y on the slice, so every plane through
// the hub maps to a line and n . e <= 0 maps to a half plane
pair Gnomonic(triple n)
{
  return (n.x / n.y, n.z / n.y);
}

// Half plane of spoke i on the unit slice: dot(p, u) <= offset
pair SpokeDirection(int i)
{
  return (Cos(60 * i), Sin(60 * i));
}

real SpokeOffset(int i)
{
  return Tan(drops[i]);
}

// Sutherland Hodgman clip of a convex polygon against dot(p, u) <= offset
pair[] ClipPolygon(pair[] poly, pair u, real offset)
{
  pair[] result;
  int n = poly.length;
  for (int k = 0; k < n; ++k)
  {
    pair a = poly[k];
    pair b = poly[(k + 1) % n];
    real da = dot(a, u) - offset;
    real db = dot(b, u) - offset;
    if (da <= 0)
      result.push(a);
    if (da * db < 0)
      result.push(a + da / (da - db) * (b - a));
  }
  return result;
}

pair[] region = { (-10, -10), (10, -10), (10, 10), (-10, 10) };
for (int i = 0; i < spokeCount; ++i)
  region = ClipPolygon(region, SpokeDirection(i), SpokeOffset(i));

// Each face normal is perpendicular to its two spokes, so it lies on both of their lines
pair[] faceNormals;
for (int i = 0; i < spokeCount; ++i)
{
  triple n = cross(spokes[(i + 1) % spokeCount], spokes[i]);
  if (n.y < 0)
    n = -n;
  faceNormals.push(Gnomonic(n));
}

// A face normal leans past the gully it borders, so sort by angle before drawing the hull
pair[] hullPoints = sort(faceNormals, new bool(pair a, pair b) { return degrees(a, false) < degrees(b, false); });

// End points of the line of spoke i, clipped to a disk of the given radius on the unit slice
pair[] SpokeLine(int i, real radius)
{
  pair u = SpokeDirection(i);
  pair t = (-u.y, u.x);
  real c = SpokeOffset(i);
  real half = sqrt(radius * radius - c * c);
  return new pair[] { c * u - half * t, c * u + half * t };
}

pen Shade(path3 face)
{
  triple n = unit(cross(point(face, 1) - point(face, 0), point(face, 2) - point(face, 0)));
  real k = 0.74 + 0.26 * abs(dot(n, lightDirection));
  return rgb(k * 0.843, k * 0.890, k * 0.957);
}

triple Lift(pair p)
{
  return sliceHeight * (p.x, 1, p.y);
}

frame[] panels;
string[] titles;

// 3D view: the fan, the slice above it and the cone of the region
{
  currentprojection = orthographic((3.4, 3.0, 2.6), up = Y);
  picture pic;
  size(pic, 250px, 250px);
  triple hub = (0, 0, 0);

  for (int i = 0; i < spokeCount; ++i)
  {
    path3 t = hub -- spokes[i] -- spokes[(i + 1) % spokeCount] -- cycle;
    draw(pic, surface(t), Shade(t), nolight);
    draw(pic, t, meshPen);
  }

  triple offset[] = {(0.1, 0, 0.1), (0.3, 0, 0.3), (0, 0, 0), (-0.1, 0, 0), (0, 0, -0.3), (0, 0, 0)};
  for (int i = 0; i < spokeCount; ++i)
  {
    draw(pic, hub -- spokes[i], SpokePen(i) + 1.4px);
    label(pic, "$e_" + string(i) + "$", spokes[i] + offset[i], unit(Gnomonic((spokes[i].x, 1, 4 * spokes[i].z))), SpokePen(i)); 
  }

  // The boundary plane of one ridge spoke, it holds the hub and its line on the slice
  pair[] line0 = SpokeLine(0, Tan(40));
  path3 wall = hub -- Lift(line0[0]) -- Lift(line0[1]) -- cycle;
  draw(pic, surface(wall), ridgeColor + opacity(0.18), nolight);
  draw(pic, hub -- Lift(line0[0]), ridgeColor + 0.5px + linetype(new real[] {4, 3}));
  draw(pic, hub -- Lift(line0[1]), ridgeColor + 0.5px + linetype(new real[] {4, 3}));

  path3 slice = shift((0, sliceHeight, 0)) * rotate(90, X) * scale3(sliceRadius) * unitcircle3;
  draw(pic, surface(slice), sliceFill, nolight);
  draw(pic, slice, gray(0.6) + 0.5px);

  for (int i = 0; i < spokeCount; ++i)
  {
    pair[] ends = SpokeLine(i, Tan(40));
    draw(pic, Lift(ends[0]) -- Lift(ends[1]), SpokePen(i) + 0.8px);
  }

  path3 top = Lift(region[0]);
  for (int k = 1; k < region.length; ++k)
    top = top -- Lift(region[k]);
  top = top -- cycle;
  draw(pic, surface(top), regionFill, nolight);
  draw(pic, top, regionPen);

  for (pair p : region)
    draw(pic, hub -- Lift(p), regionPen);

  dot(pic, hub, ink + 3px);
  label(pic, "slice", Lift((0, -Tan(40))) + (0, 0.1, 0), N, gray(0.35));

  panels.push(pic.fit());
  titles.push("Saddle Vertex Voronoi");
}

// Top view of the slice, in units of tan(tilt)
{
  picture pic;
  unitsize(pic, 190px);
  real radius = Tan(40);

  // Looking down on the slice: x to the right, z down the page
  pair View(pair p)
  {
    return (p.x, -p.y);
  }

  path hull = View(hullPoints[0]);
  for (int i = 1; i < spokeCount; ++i)
    hull = hull -- View(hullPoints[i]);
  hull = hull -- cycle;
  fill(pic, hull, gray(0.92));

  // Excluded side of each line, a band inside the disk
  real band = 0.035;
  for (int i = 0; i < spokeCount; ++i)
  {
    pair u = SpokeDirection(i);
    pair[] ends = SpokeLine(i, radius - band);
    path strip = View(ends[0]) -- View(ends[1]) -- View(ends[1] + band * u) -- View(ends[0] + band * u) -- cycle;
    fill(pic, strip, SpokePen(i) + opacity(0.25));
  }

  for (int i = 0; i < spokeCount; ++i)
  {
    pair[] ends = SpokeLine(i, radius);
    draw(pic, View(ends[0]) -- View(ends[1]), SpokePen(i) + 1.0px);
    pair u = SpokeDirection(i);
    label(pic, "$e_" + string(i) + "$", View((SpokeOffset(i) + (IsRidge(i) ? 0.11 : 0.075)) * u), SpokePen(i));
  }

  draw(pic, hull, hullPen);

  path area = View(region[0]);
  for (int k = 1; k < region.length; ++k)
    area = area -- View(region[k]);
  area = area -- cycle;
  filldraw(pic, area, regionFill, regionPen);

  // Tilt from up, drawn over the fills so they read through the region
  for (real tilt : new real[] {10, 20, 30, 40})
  {
    draw(pic, scale(Tan(tilt)) * unitcircle, tilt == 40 ? gray(0.6) + 0.5px : guidePen);
    label(pic, string(tilt) + "$^\circ$", Tan(tilt) * dir(90), N, gray(0.45) + fontsize(7pt));
  }

  for (pair n : faceNormals)
    dot(pic, View(n), ink + 3.5px);
  label(pic, "face normals", View(faceNormals[1]), NE, ink + fontsize(8pt));

  dot(pic, (0, 0), ink + 2.5px);
  label(pic, "up", (0, 0), SW, ink + fontsize(8pt));

  frame f = pic.fit();
  panels.push(f);
  titles.push("Top View");
}

real gap = 36px;
real x = 0;
real bottom = 0;
for (int i = 0; i < panels.length; ++i)
{
  frame f = panels[i];
  pair lo = min(f);
  pair hi = max(f);
  add(shift(x - lo.x, -lo.y) * f);
  label(titles[i], (x + 0.5 * (hi.x - lo.x), -8px), S, ink);
  x += hi.x - lo.x + gap;
}

// Legend
pair legend = (0, -40px);
draw((legend.x, legend.y) -- (legend.x + 18px, legend.y), ridgeColor + 1.4px);
label("ridge spoke, 8$^\circ$ drop", (legend.x + 22px, legend.y), E, ink + fontsize(8pt));
draw((legend.x + 150px, legend.y) -- (legend.x + 168px, legend.y), gullyColor + 1.4px);
label("gully spoke, 32$^\circ$ drop", (legend.x + 172px, legend.y), E, ink + fontsize(8pt));
filldraw(shift(legend.x + 300px, legend.y - 5px) * scale(10px) * unitsquare, regionFill, regionPen);
label("vertex region", (legend.x + 314px, legend.y), E, ink + fontsize(8pt));
draw((legend.x + 410px, legend.y) -- (legend.x + 428px, legend.y), hullPen);
label("cone of face normals", (legend.x + 432px, legend.y), E, ink + fontsize(8pt));
