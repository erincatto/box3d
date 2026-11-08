settings.outformat = "svg";
settings.render = 0;
settings.prc = false;

import three;

real px = 0.75bp;

pen ink = rgb("231f20");
pen meshPen = ink + 0.6px;
pen regionFill = rgb("f2d4d0");
pen regionPen = rgb("a8322a") + 0.9px;
pen rayPen = rgb("a8322a") + 0.6px;
pen spokePen = rgb("1f5c99") + 1.0px;
pen cliffFill = gray(0.88) + opacity(0.6);
pen guidePen = gray(0.45) + 0.6px + linetype(new real[] {4, 3});

real panelSize = 170px;
real capRadius = 0.55;
real arrowSize = 5px;

triple lightDirection = unit((1, 4, 2));

// Extreme rays of the cone of normals n with n . d <= 0 for every direction d
triple[] RegionRays(triple[] d)
{
  triple[] rays;
  for (int i = 0; i < d.length; ++i)
  {
    for (int j = i + 1; j < d.length; ++j)
    {
      triple c = cross(d[i], d[j]);
      if (abs(c) < 1e-6)
        continue;

      for (real s : new real[] {1, -1})
      {
        triple r = s * unit(c);
        bool inside = true;
        for (triple e : d)
          inside = inside && dot(r, unit(e)) <= 1e-6;

        bool duplicate = false;
        for (triple q : rays)
          duplicate = duplicate || dot(r, q) > 1 - 1e-6;

        if (inside && !duplicate)
          rays.push(r);
      }
    }
  }

  triple axis = (0, 0, 0);
  for (triple r : rays)
    axis += r;
  axis = unit(axis);

  triple t1 = unit(cross(axis, abs(axis.x) < 0.9 ? X : Y));
  triple t2 = cross(axis, t1);
  real[] angles;
  for (triple r : rays)
    angles.push(atan2(dot(r, t2), dot(r, t1)));

  int[] order = sequence(rays.length);
  order = sort(order, new bool(int a, int b) { return angles[a] < angles[b]; });
  triple[] sorted;
  for (int k : order)
    sorted.push(rays[k]);
  return sorted;
}

triple Arc(triple a, triple b, real t)
{
  return unit((1 - t) * a + t * b);
}

// Extreme rays are face or cliff normals, drawn as arrows leaving the rim
void DrawRayTips(picture pic, triple v, triple[] rays, real radius)
{
  for (triple r : rays)
    draw(pic, (v + radius * r) -- (v + (radius + 0.3) * r), regionPen, Arrow3(arrowSize));
}

// The region is a convex cone, so drawing only the faces that face the camera needs no sorting
void DrawRegion(picture pic, triple v, triple[] rays, real radius)
{
  int arcSteps = 12;
  int n = rays.length;
  triple view = unit(currentprojection.camera - currentprojection.target);

  if (n == 2)
  {
    path3 arc = v + radius * rays[0];
    for (int k = 1; k <= arcSteps; ++k)
      arc = arc -- (v + radius * Arc(rays[0], rays[1], k / arcSteps));
    draw(pic, surface(v -- arc -- cycle), regionFill, nolight);
    draw(pic, arc, regionPen);
    for (triple r : rays)
      draw(pic, v -- (v + radius * r), regionPen);
    DrawRayTips(pic, v, rays, radius);
    return;
  }

  triple axis = (0, 0, 0);
  for (triple r : rays)
    axis += r;
  axis = unit(axis);

  int rings = 6;
  surface cap;
  path3 rim;
  for (int i = 0; i < n; ++i)
  {
    triple a = rays[i];
    triple b = rays[(i + 1) % n];

    path3 arc = v + radius * a;
    for (int k = 1; k <= arcSteps; ++k)
      arc = arc -- (v + radius * Arc(a, b, k / arcSteps));

    triple side = cross(a, b);
    if (dot(side, axis) > 0)
      side = -side;
    if (dot(side, view) > 0)
    {
      draw(pic, surface(v -- arc -- cycle), regionFill, nolight);
      draw(pic, v -- arc -- cycle, rayPen);
    }

    for (int k = 0; k < arcSteps; ++k)
    {
      triple p0 = Arc(a, b, k / arcSteps);
      triple p1 = Arc(a, b, (k + 1) / arcSteps);
      rim = i == 0 && k == 0 ? v + radius * p0 : rim -- (v + radius * p0);
      for (int j = 0; j < rings; ++j)
      {
        real u0 = j / rings;
        real u1 = (j + 1) / rings;
        triple q00 = Arc(axis, p0, u0);
        triple q10 = Arc(axis, p0, u1);
        triple q11 = Arc(axis, p1, u1);
        triple q01 = Arc(axis, p1, u0);
        if (dot(q00 + q11, view) > 0)
          cap.append(surface(v + radius * q00 -- v + radius * q10 -- v + radius * q11 -- v + radius * q01 -- cycle));
      }
    }
  }

  // The mesh pen covers the hairline seams between patches
  draw(pic, cap, surfacepen = regionFill, meshpen = regionFill + 0.5px, nolight);
  draw(pic, rim -- cycle, regionPen);
  DrawRayTips(pic, v, rays, radius);
}

// Flat shading in world space, so every panel is lit the same way
pen Shade(path3 face)
{
  triple n = unit(cross(point(face, 1) - point(face, 0), point(face, 2) - point(face, 0)));
  real k = 0.74 + 0.26 * abs(dot(n, lightDirection));
  return rgb(k * 0.843, k * 0.890, k * 0.957);
}

void DrawFan(picture pic, triple v, triple[] rim, bool closed)
{
  int count = closed ? rim.length : rim.length - 1;
  for (int i = 0; i < count; ++i)
  {
    path3 t = v -- rim[i] -- rim[(i + 1) % rim.length] -- cycle;
    draw(pic, surface(t), Shade(t), nolight);
    draw(pic, t, meshPen);
  }
}

void DrawSpokes(picture pic, triple v, triple[] ends)
{
  for (triple e : ends)
    draw(pic, v -- (v + 0.45 * (e - v)), spokePen, Arrow3(arrowSize));
}

frame[] panels;
string[] titles;

// Box corner, three faces meet at a convex vertex
{
  currentprojection = orthographic((3.5, 2.6, 2.8), up = Y);
  picture pic;
  size(pic, panelSize, panelSize);
  triple v = (0, 0, 0);

  path3[] faces = {
    (0, 0, 0) -- (0, -1, 0) -- (0, -1, -1) -- (0, 0, -1) -- cycle,
    (0, 0, 0) -- (0, 0, -1) -- (-1, 0, -1) -- (-1, 0, 0) -- cycle,
    (0, 0, 0) -- (-1, 0, 0) -- (-1, -1, 0) -- (0, -1, 0) -- cycle,
  };
  for (path3 f : faces)
  {
    draw(pic, surface(f), Shade(f), nolight);
    draw(pic, f, meshPen);
  }

  triple[] ends = { (-1, 0, 0), (0, -1, 0), (0, 0, -1) };
  DrawSpokes(pic, v, ends);
  DrawRegion(pic, v, RegionRays(ends), capRadius);
  dot(pic, v, ink + 3px);

  panels.push(pic.fit());
  titles.push("convex corner");
}

// A vertex on a ridge, the region collapses to a flat wedge
{
  currentprojection = orthographic((3.0, 2.4, 2.2), up = Y);
  picture pic;
  size(pic, panelSize, panelSize);
  triple v = (0, 0, 0);
  real h = tan(30 * pi / 180);

  triple[] rim = { (1, 0, 0), (1, -h, 1), (-1, -h, 1), (-1, 0, 0), (-1, -h, -1), (1, -h, -1) };
  DrawFan(pic, v, rim, true);
  DrawSpokes(pic, v, rim);
  DrawRegion(pic, v, RegionRays(rim), capRadius);
  dot(pic, v, ink + 3px);

  panels.push(pic.fit());
  titles.push("ridge");
}

// Saddle hub, spokes alternate 8 degree ridges and 32 degree gullies
{
  currentprojection = orthographic((2.0, 4.0, 3.0), up = Y);
  picture pic;
  size(pic, panelSize, panelSize);
  triple v = (0, 0, 0);

  triple[] rim;
  for (int i = 0; i < 6; ++i)
  {
    real angle = i * 60;
    real drop = i % 2 == 1 ? 32 : 8;
    rim.push((Cos(angle), -Tan(drop), Sin(angle)));
  }

  DrawFan(pic, v, rim, true);
  DrawSpokes(pic, v, rim);
  DrawRegion(pic, v, RegionRays(rim), 1.2);
  dot(pic, v, ink + 3px);

  panels.push(pic.fit());
  titles.push("saddle hub");
}

// Corner of an open sheet, the cliffs below the boundary edges close the region
{
  currentprojection = orthographic((3.5, 2.6, 2.8), up = Y);
  picture pic;
  size(pic, panelSize, panelSize);
  triple v = (0, 0, 0);
  real depth = 0.6;

  path3[] cliffs = {
    (0, 0, 0) -- (0, 0, -1) -- (0, -depth, -1) -- (0, -depth, 0) -- cycle,
    (0, 0, 0) -- (-1, 0, 0) -- (-1, -depth, 0) -- (0, -depth, 0) -- cycle,
  };
  for (path3 c : cliffs)
  {
    draw(pic, surface(c), cliffFill, nolight);
    draw(pic, c, guidePen);
  }

  triple[] rim = { (-1, 0, 0), (-1, 0, -1), (0, 0, -1) };
  DrawFan(pic, v, rim, false);
  DrawSpokes(pic, v, rim);

  triple[] directions = copy(rim);
  directions.push((0, -1, 0));
  DrawRegion(pic, v, RegionRays(directions), capRadius);
  dot(pic, v, ink + 3px);
  label(pic, "cliff", (0, -depth, -0.5), S, gray(0.35));

  panels.push(pic.fit());
  titles.push("open corner");
}

real gap = 24px;
real x = 0;
for (int i = 0; i < panels.length; ++i)
{
  frame f = panels[i];
  pair lo = min(f);
  pair hi = max(f);
  add(shift(x - lo.x, -lo.y) * f);
  label(titles[i], (x + 0.5 * (hi.x - lo.x), -8px), S, ink);
  x += hi.x - lo.x + gap;
}
