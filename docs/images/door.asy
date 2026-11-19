settings.outformat = "svg";
settings.render = 0;

real px = 0.75bp;

pen ink = rgb("231f20");
pen edgePen = ink + 0.8px;
pen thinPen = ink + 0.5px;
pen hatchPen = gray(0.55) + 0.4px;
pen bodyFill = rgb("d7e3f4");
pen plateFill = gray(0.88);
pen goodPen = rgb("1f5c99") + 1.0px;
pen badPen = rgb("a8322a") + 1.0px;
pen guidePen = gray(0.5) + 0.6px + linetype(new real[] {4, 3});
pen small = fontsize(8pt);

real scaleUnit = 80px;
real arrowSize = 5px;
real dimensionArrow = 4px;

// Door width exceeds the clear opening, exaggerated so the overlap reads
real doorWidth = 2.0;
real opening = 1.82;
real wallThickness = 0.26;
real doorThickness = 0.08;
real wallEnd = -0.35;
real wallLength = 0.9;
real pinRadius = 0.045;
real openAngle = 28;
real play = doorWidth - opening;

void Hatch(picture pic, path region, real spacing = 0.06)
{
  picture h;
  pair lo = min(region);
  pair hi = max(region);
  real height = hi.y - lo.y;
  for (real s = 0; s <= hi.x - lo.x + height; s += spacing)
    draw(h, (lo.x + s, lo.y) -- (lo.x + s - height, hi.y), hatchPen);
  clip(h, region);
  add(pic, h);
}

path BreakLine(real x)
{
  real half = 0.5 * wallThickness;

// Both panels share one coordinate system so the walls line up
unitsize(scaleUnit);
real panelOffset = 4.5;
real titleX = 0.5 * (wallEnd - wallLength + opening + wallLength);
real titleY = -0.85;
  real e = 0.04;
  return (x, -half - e) -- (x, -e) -- (x - e, -0.3 * e) -- (x + e, 0.3 * e) -- (x, e) -- (x, half + e);
}

void DrawWalls(picture pic)
{
  real half = 0.5 * wallThickness;

// Both panels share one coordinate system so the walls line up
unitsize(scaleUnit);
real panelOffset = 4.5;
real titleX = 0.5 * (wallEnd - wallLength + opening + wallLength);
real titleY = -0.85;
  real far = wallEnd - wallLength;
  path hingeWall = box((far, -half), (wallEnd, half));
  path jamb = box((opening, -half), (opening + wallLength, half));
  for (path wall : new path[] {hingeWall, jamb})
  {
    fill(pic, wall, white);
    Hatch(pic, wall);
  }

  // Cut faces are solid, the far ends get break lines
  draw(pic, (far, half) -- (wallEnd, half) -- (wallEnd, -half) -- (far, -half), edgePen);
  draw(pic, (opening + wallLength, half) -- (opening, half) -- (opening, -half) -- (opening + wallLength, -half), edgePen);
  draw(pic, BreakLine(far), thinPen);
  draw(pic, BreakLine(opening + wallLength), thinPen);

  label(pic, "jamb", (opening + 0.5 * wallLength, -half), S, ink + small);
}

void DrawPlate(picture pic)
{
  real lug = 0.075;
  path plate = (wallEnd, -lug) -- (0, -lug) -- arc((0, 0), lug, -90, 90) -- (wallEnd, lug) -- cycle;
  filldraw(pic, plate, plateFill, thinPen);
}

path Leaf(pair pin, real angle)
{
  pair u = dir(angle);
  pair n = rotate(90) * u;
  real h = 0.5 * doorThickness;
  return (pin - h * n) -- (pin + doorWidth * u - h * n) -- (pin + doorWidth * u + h * n) -- (pin + h * n) -- cycle;
}

void DrawPin(picture pic, pair pin)
{
  filldraw(pic, circle(pin, pinRadius), white, edgePen);
  dot(pic, pin, ink + 1.5px);
}

void Dimension(picture pic, pair a, pair b, real offset, string text, pair labelShift = (0, 0))
{
  pair n = rotate(90) * unit(b - a);
  pair a1 = a + offset * n;
  pair b1 = b + offset * n;
  real s = offset > 0 ? 1 : -1;
  draw(pic, a + 0.08 * s * n -- a1 + 0.06 * s * n, thinPen);
  draw(pic, b + 0.08 * s * n -- b1 + 0.06 * s * n, thinPen);
  draw(pic, a1 -- b1, thinPen, Arrows(dimensionArrow));
  label(pic, text, 0.5 * (a1 + b1) + labelShift, s * n, ink + small);
}

void Leader(picture pic, pair a, pair b, string text, pen p = ink)
{
  draw(pic, a -- b, p + 0.5px);
  label(pic, text, b, b.x < a.x ? W : E, p + small);
}

real half = 0.5 * wallThickness;

// Both panels share one coordinate system so the walls line up
unitsize(scaleUnit);
real panelOffset = 4.5;
real titleX = 0.5 * (wallEnd - wallLength + opening + wallLength);
real titleY = -0.85;

// Hinge pinned, the latch edge only moves on its arc and the arc runs into the jamb
{
  picture pic;
  pair pin = (0, 0);

  DrawWalls(pic);

  real cornerAngle = aSin(half / doorWidth);
  path interference = (opening, -half) -- (opening, half) -- arc(pin, doorWidth, cornerAngle, -cornerAngle) -- cycle;
  fill(pic, interference, badPen + opacity(0.25));
  draw(pic, interference, badPen + 0.6px);
  draw(pic, arc(pin, doorWidth, openAngle, -cornerAngle), guidePen);
  Leader(pic, (opening + 0.08, 0.06), (opening + 0.45, 0.45), "interference", rgb("a8322a"));

  DrawPlate(pic);
  filldraw(pic, Leaf(pin, openAngle), bodyFill, edgePen);
  DrawPin(pic, pin);

  path swing = arc(pin, 1.4, openAngle - 3, 5);
  draw(pic, swing, goodPen, Arrow(arrowSize));
  label(pic, "swing", endpoint(swing), W, goodPen + small);

  // Swinging never moves the latch edge toward the hinge
  pair u = dir(openAngle);
  pair edge = pin + doorWidth * u;
  pair start = edge + 0.5 * u;
  pair end = edge + 0.07 * u;
  draw(pic, start -- end, badPen, Arrow(arrowSize));
  pair mid = 0.5 * (start + end);
  pair s = rotate(90) * u;
  real crossSize = 0.05;
  draw(pic, mid - crossSize * (u + s) -- mid + crossSize * (u + s), badPen + 1.2px);
  draw(pic, mid - crossSize * (u - s) -- mid + crossSize * (u - s), badPen + 1.2px);

  Dimension(pic, pin, edge, 0.38, "$r$");
  Dimension(pic, (0, -half), (opening, -half), -0.42, "$w$");
  Leader(pic, pin + 0.06 * dir(225), (-0.5, -0.3), "hinge pin");

  add(pic);
  label("zero inverse mass", (titleX, titleY), S, ink);
}

// Hinge pin in a slot, it gives by r minus w and the door seats in the frame
{
  picture pic;
  pair pin = (-play, 0);

  DrawWalls(pic);
  DrawPlate(pic);

  real slotRadius = pinRadius + 0.012;
  path slot = (-play, -slotRadius) -- (0, -slotRadius) -- arc((0, 0), slotRadius, -90, 90) -- (-play, slotRadius) -- arc((-play, 0), slotRadius, 90, 270) -- cycle;
  filldraw(pic, slot, white, thinPen);

  filldraw(pic, Leaf(pin, 0), bodyFill, edgePen);
  draw(pic, circle((0, 0), pinRadius), guidePen);
  DrawPin(pic, pin);

  Dimension(pic, (pin.x, half), (opening, half), 0.45, "$r$");
  Dimension(pic, (pin.x, -half), (0, -half), -0.42, "$r - w$", (-0.3, 0));
  Dimension(pic, (0, -half), (opening, -half), -0.42, "$w$");
  Leader(pic, (-play - 0.03, -0.05), (-0.5, -0.3), "flexible hinge");

  add(shift(panelOffset, 0) * pic);
  label("motion lock joint", (panelOffset + titleX, titleY), S, ink);
}
