settings.outformat = "svg";
settings.render = 0;

real px = 0.75bp;

pen ink = rgb("231f20");
pen bodyFill = rgb("d7e3f4");
pen bodyPen = ink + 0.8px;
pen goodPen = rgb("1f5c99") + 1.0px;
pen badPen = rgb("a8322a") + 1.0px;
pen guidePen = gray(0.5) + 0.6px + linetype(new real[] {4, 3});
pen lockPen = gray(0.45) + 0.8px;
pen small = fontsize(8pt);

real scaleUnit = 52px;
real arrowSize = 5px;
real tilt = 15;
real halfWidth = 1.0;
real halfHeight = 0.5;
real armLength = 0.72;
real arrowLength = 0.6;

void DrawBody(picture pic)
{
  path box = rotate(tilt) * box((-halfWidth, -halfHeight), (halfWidth, halfHeight));
  filldraw(pic, box, bodyFill, bodyPen);
}

// The usual center of mass symbol, a circle with two filled quadrants
void DrawCenter(picture pic)
{
  real radius = 0.085;
  fill(pic, (0, 0) -- arc((0, 0), radius, tilt, tilt + 90) -- cycle, ink);
  fill(pic, (0, 0) -- arc((0, 0), radius, tilt + 180, tilt + 270) -- cycle, ink);
  draw(pic, circle((0, 0), radius), ink + 0.6px);
}

pair Pivot()
{
  return rotate(tilt) * (armLength, 0);
}

void DrawPivot(picture pic, pair p)
{
  filldraw(pic, circle(p, 0.06), white, ink + 0.8px);
}

// Rotation about the center moves the pivot along the circle, never toward or away from the center
void DrawArm(picture pic)
{
  pair p = Pivot();
  draw(pic, circle((0, 0), armLength), guidePen);
  draw(pic, (0, 0) -- p, goodPen + 0.7px, Arrow(arrowSize), Margins);
  label(pic, "$\mathbf{r}$", 0.5 * p, rotate(90) * unit(p), ink + small);

  pair tangent = rotate(90) * unit(p);
  draw(pic, p -- p + arrowLength * tangent, goodPen, Arrow(arrowSize));
  label(pic, "$\omega \times \mathbf{r}$", p + arrowLength * tangent, N, ink + small);
}

frame[] panels;
string[] titles;

// Translation locked by zeroing the inverse mass, the pivot cannot move along the arm
{
  picture pic;
  unitsize(pic, scaleUnit);
  DrawBody(pic);
  DrawArm(pic);

  pair p = Pivot();
  pair radial = unit(p);
  pair tip = p + arrowLength * radial;
  draw(pic, p -- tip, badPen, Arrow(arrowSize));

  pair mid = p + 0.55 * arrowLength * radial;
  pair side = rotate(90) * radial;
  real crossSize = 0.1;
  draw(pic, mid - crossSize * (radial + side) -- mid + crossSize * (radial + side), badPen + 1.2px);
  draw(pic, mid - crossSize * (radial - side) -- mid + crossSize * (radial - side), badPen + 1.2px);
  label(pic, "$\hat{\mathbf{r}}^T K \hat{\mathbf{r}} = 0$", tip, SE, badPen + small);

  DrawPivot(pic, p);
  DrawCenter(pic);
  label(pic, "$m^{-1} = 0$", (0, -0.2), S, ink + small);

  panels.push(pic.fit());
  titles.push("zero inverse mass");
}

// Impulse needed to remove a position error along the arm
{
  picture pic;
  unitsize(pic, scaleUnit);

  real xScale = 1.6;
  real error = 0.22;
  real top = 1.8;
  real kMax = 1.3;
  real kLock = 0.75;

  path curve;
  int count = 64;
  real kMin = error / top;
  for (int i = 0; i <= count; ++i)
  {
    real k = kMin + (kMax - kMin) * i / count;
    pair q = (xScale * k, error / k);
    curve = i == 0 ? q : curve -- q;
  }

  fill(pic, box((xScale * kLock, 0), (xScale * kMax, top)), bodyFill + opacity(0.6));
  draw(pic, (0, 0) -- (xScale * kMax + 0.15, 0), ink + 0.7px, Arrow(arrowSize));
  draw(pic, (0, 0) -- (0, top + 0.2), ink + 0.7px, Arrow(arrowSize));
  draw(pic, curve, ink + 1.0px);

  pair lockPoint = (xScale * kLock, error / kLock);
  dot(pic, lockPoint, goodPen + 4px);
  label(pic, "$k \ge m^{-1}$", lockPoint, NE, goodPen + small);

  draw(pic, (0.55, top - 0.1) -- (0.12, top - 0.1), badPen, Arrow(arrowSize));
  label(pic, "$k \rightarrow 0$", (0.55, top - 0.1), E, badPen + small);

  label(pic, "$k = \hat{\mathbf{r}}^T K \hat{\mathbf{r}}$", (xScale * kMax + 0.15, 0), S, ink + small);
  label(pic, "$|\lambda| = |C| / k$", (0, top + 0.2), E, ink + small);

  panels.push(pic.fit());
  titles.push("impulse along the arm");
}

// The lock is its own joint and the body keeps its mass, so the pivot can still move along the arm
{
  picture pic;
  unitsize(pic, scaleUnit);
  DrawBody(pic);
  DrawArm(pic);

  pair p = Pivot();
  pair radial = unit(p);
  pair tip = p + arrowLength * radial;
  draw(pic, p -- tip, goodPen, Arrow(arrowSize));
  label(pic, "$\hat{\mathbf{r}}^T K \hat{\mathbf{r}} \ge m^{-1}$", tip, SE, goodPen + small);

  DrawPivot(pic, p);

  // Frame of the motion lock joint on the ground
  real axisLength = 0.2;
  draw(pic, (-axisLength, 0) -- (axisLength, 0), lockPen);
  draw(pic, (0, -axisLength) -- (0, axisLength), lockPen);
  DrawCenter(pic);
  label(pic, "$m^{-1} > 0$", (0, -0.2), S, ink + small);

  panels.push(pic.fit());
  titles.push("motion lock joint");
}

real gap = 30px;
real x = 0;
real bottom = infinity;
for (frame f : panels)
  bottom = min(bottom, 0.5 * (min(f).y - max(f).y));

for (int i = 0; i < panels.length; ++i)
{
  frame f = panels[i];
  pair lo = min(f);
  pair hi = max(f);
  add(shift(x - lo.x, -0.5 * (lo.y + hi.y)) * f);
  label(titles[i], (x + 0.5 * (hi.x - lo.x), bottom - 8px), S, ink);
  x += hi.x - lo.x + gap;
}
