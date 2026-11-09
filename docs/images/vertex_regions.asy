settings.outformat = "svg";
settings.render = 0;
settings.prc = false;

import three;

real px = 0.75bp;

pen ink = rgb("231f20");
pen meshPen = ink + 0.6px;
// Half opacity over white matches the opaque f2d4d0 of the other figures
pen regionFill = rgb("e5a9a1") + opacity(0.5);
pen regionShade = rgb("7a1f18");
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

// Extreme rays of the region, drawn as arrows leaving the rim
void DrawRayTips(picture pic, triple v, triple[] rays, real radius)
{
  for (triple r : rays)
    draw(pic, (v + radius * r) -- (v + (radius + 0.3) * r), regionPen, Arrow3(arrowSize));
}

// Convex hull by monotone chain
pair[] ConvexHull(pair[] points)
{
  pair[] p = sort(points, new bool(pair a, pair b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  pair[] hull;
  for (int pass = 0; pass < 2; ++pass)
  {
    int base = hull.length;
    for (pair q : p)
    {
      while (hull.length >= base + 2 && cross(hull[hull.length - 1] - hull[hull.length - 2], q - hull[hull.length - 2]) <= 0)
        hull.pop();
      hull.push(q);
    }
    hull.pop();
    p = reverse(p);
  }
  return hull;
}

// The region is convex, so its outline is the hull of its projected points. Filling that once
// avoids the seams that overlapping translucent patches would show.
void FillProjected(picture pic, triple v, triple[] points)
{
  triple view = unit(currentprojection.camera - currentprojection.target);
  triple u1 = unit(cross(currentprojection.up, view));
  triple u2 = cross(view, u1);

  pair[] flat;
  for (triple q : points)
    flat.push((dot(q - v, u1), dot(q - v, u2)));

  pair[] hull = ConvexHull(flat);
  path3 outline = v + hull[0].x * u1 + hull[0].y * u2;
  for (int k = 1; k < hull.length; ++k)
    outline = outline -- (v + hull[k].x * u1 + hull[k].y * u2);
  draw(pic, surface(outline -- cycle, planar = true), regionFill, nolight);
}

void DrawRegion(picture pic, triple v, triple[] rays, real radius)
{
  int arcSteps = 12;
  int n = rays.length;

  triple axis = (0, 0, 0);
  for (triple r : rays)
    axis += r;
  axis = unit(axis);

  triple[] points = { v };
  path3 rim;
  for (int i = 0; i < n; ++i)
  {
    if (n == 2 && i == 1)
      break;

    triple a = rays[i];
    triple b = rays[(i + 1) % n];
    for (int k = 0; k <= arcSteps; ++k)
    {
      triple p = v + radius * Arc(a, b, k / arcSteps);
      rim = i == 0 && k == 0 ? p : rim -- p;
      points.push(p);

      // Interior points for the silhouette of the spherical cap
      for (int j = 1; j < 6 && n > 2; ++j)
        points.push(v + radius * Arc(axis, Arc(a, b, k / arcSteps), j / 6));
    }
  }

  FillProjected(pic, v, points);

  // Darken the flat sides with the same light as the mesh. Sides on the near half of the cone never
  // overlap each other in projection, nor do sides on the far half, so each adds one layer.
  for (int i = 0; i < n; ++i)
  {
    if (n == 2 && i == 1)
      break;

    triple a = rays[i];
    triple b = rays[(i + 1) % n];
    triple side = unit(cross(a, b));

    path3 sector = v + radius * a;
    for (int k = 1; k <= arcSteps; ++k)
      sector = sector -- (v + radius * Arc(a, b, k / arcSteps));
    sector = v -- sector -- cycle;

    real shade = 1 - abs(dot(side, lightDirection));
    draw(pic, surface(sector, planar = true), regionShade + opacity(0.35 * shade), nolight);
  }

  draw(pic, n == 2 ? rim : rim -- cycle, regionPen);
  for (triple r : rays)
    draw(pic, v -- (v + radius * r), rayPen);
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
