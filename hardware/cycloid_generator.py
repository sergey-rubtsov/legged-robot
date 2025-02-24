# Has been stolen here: https://github.com/RepRapLtd/RobotComponents
#
# Cycloidal drive creation program
#
# RepRap Ltd
# https://reprapltd.com
#
# Written by Adrian Bowyer
# 30 September 2021
#
# Licence: GPL
#
# This builds all the basic geometry needed for a cycloidal drive.
#
# Note - it makes exact geometry*; it doesn't offset shapes to allow clearances, so you probably want to
# do things like saying the pins and rollers are 0.1mm bigger than they actually are - experiment.
#
# *Well, exact: the cycloidal disc it creates is a many-faceted approximation to the true algebra.
# The faceting can be made as fine as you like though.
#

import math as maths
import Part
from FreeCAD import Base

# *****************************************************************************************************************************

# Put numbers in here.
# See https://www.tec-science.com/mechanical-power-transmission/planetary-gear/construction-of-the-cycloidal-disc/
# All dimensions are in mm.

D = 45  # Pin circle centres diameter

n = 9  # Number of lobes
N = 10  # Number of pins

central_axis_radius = 1.5  # Central input shaft radius
d_shaft = 0.3 * central_axis_radius  # D-flat on shaft

# Small bearing 3x6x2.5MM ABEC 7 High-Speed MR63 ZZ
small_bearing_external_diameter = 6.0
small_bearing_internal_diameter = 3.0
small_bearing_length = 2.5

# Eccentric bearing
eccentric_bearing_external_diameter = 15.0
eccentric_bearing_internal_diameter = 10.0

# Ring bearing 6806-2RS 30X42X7mm:
ring_bearing_internal_diameter = 30.0
ring_bearing_external_diameter = 42.0
ring_bearing_length = 7.0

ring_length = 6.0

cycloid_length = small_bearing_length  # 3mm How thick to make the contracted cycloidal disc
dr = small_bearing_external_diameter  # 6mm Inner roller pin diameter (bearing diameter)
dp = small_bearing_external_diameter  # 6mm Pin diameter (bearing diameter)
r_pin_axis = small_bearing_internal_diameter / 2  # 1.5mm Pin axis radius
holes = 3  # Number of roller holes (must equal n or divide into n exactly)
bearing_stop_length = 2.0  # Thickness of stop for ring and roller bearings
dd = 25  # Inner roller pin centres diameter
ddd = ring_bearing_internal_diameter  # 30mm inner roller diameter, should be equal to inner bearing diameter
eFactor = 0.3  # Sets the eccentricity. Must be less than 0.5.
# If it's too big the faceting will be inaccurate, so increase circle below.
dc = eccentric_bearing_external_diameter  # 15mm Diameter of central hole

lip = -1  # Create a lip to constrain axial drift. Set to -1 to supress.
circle = 800  # Sets the faceting to correspond to 1 degree; a 360-faced polygon.
# Bigger for finer faceting (use even numbers).

# ***************************************************************************************************************************

# Work out some parameters and tell the user
print("----------------------------------\n")
i = n / (N - n)
print("Transmission ratio: " + str(i))
delta = D / N
print("Rolling circle diameter: " + str(delta))
d = i * D / N
print("Base circle diameter: " + str(d))
e = delta * eFactor
print("Eccentricity: " + str(e))
eccentric_diameter = central_axis_radius * 2 + e
print("Eccentric diameter: " + str(eccentric_diameter))
dh = dr + 2 * e
print("Hole diameter: " + str(dh) + ". There are " + str(holes) + " on pitch diameter " + str(dd) + ".")


# This is nasty...
def null_set():
    a = Part.makeBox(1, 1, 1)
    a.translate(Base.Vector(10, 10, 10))
    return a.common(Part.makeBox(1, 1, 1))


# Compute a normal vector length r using a chord across a curve (i.e. 2nd degree approximation).
def normal_vector(xOld, xNew, yOld, yNew, r):
    dy = xNew - xOld
    dx = yOld - yNew
    s = r / maths.sqrt(dx * dx + dy * dy)
    return dx * s, dy * s


def roller_hole():
    roller = Part.makeCylinder(dh / 2, cycloid_length, Base.Vector(0, 0, 0), Base.Vector(0, 0, 1))
    if lip > 0:
        roller = roller.fuse(
            Part.makeCone(dh / 2, dh / 2 - lip, lip, Base.Vector(0, 0, cycloid_length), Base.Vector(0, 0, 1)))
        roller = roller.fuse(Part.makeCone(dh / 2, dh / 2 - lip, lip, Base.Vector(0, 0, 0), Base.Vector(0, 0, -1)))
        roller = roller.fuse(
            Part.makeCylinder(dh / 2 - lip, lip, Base.Vector(0, 0, cycloid_length + lip), Base.Vector(0, 0, 1)))
        roller = roller.fuse(Part.makeCylinder(dh / 2 - lip, lip, Base.Vector(0, 0, -lip), Base.Vector(0, 0, -1)))
    return roller


# This builds the blank contracted cycloid.
# Note we use the first three of times round the loop
# to get the numbers for the curve-normal calculation,
# so we have to go +3 at the end to join up.
def contracted_cycloid_blank():
    offsetCycloid = []
    if lip > 0:
        lipWire = []
    x = d / 2 + delta / 2
    y = 0
    circle_pi = circle * 0.5
    for theta in range(circle + 3):
        xNew = ((d + delta) / 2) * maths.cos(theta * maths.pi / circle_pi)
        yNew = ((d + delta) / 2) * maths.sin(theta * maths.pi / circle_pi)
        phi = d * theta / delta
        xNew += e * maths.cos((theta + phi) * maths.pi / circle_pi)
        yNew += e * maths.sin((theta + phi) * maths.pi / circle_pi)
        if theta >= 2:
            dxy = normal_vector(xOld, xNew, yOld, yNew, dp / 2)
            offsetCycloid.append(App.Vector(x + dxy[0], y + dxy[1], 0))
            if lip > 0:
                dxy = normal_vector(xOld, xNew, yOld, yNew, dp / 2 - lip)
                lipWire.append(App.Vector(x + dxy[0], y + dxy[1], 0))
        xOld = x
        yOld = y
        x = xNew
        y = yNew
    offset_cycloid_wire = Part.makePolygon(offsetCycloid)
    face = Part.Face(offset_cycloid_wire)
    result = face.extrude(Base.Vector(0, 0, cycloid_length))
    if lip > 0:
        lipWire = Part.makePolygon(lipWire)
        lipWire.translate(Base.Vector(0, 0, -lip))
        lipFace1 = Part.Face(lipWire)
        lipExtrude1 = lipFace1.extrude(Base.Vector(0, 0, -lip))
        lipExtrude2 = lipExtrude1.copy().translate(Base.Vector(0, 0, cycloid_length + 3 * lip))
        loft = [lipWire, offset_cycloid_wire]
        loft = Part.makeLoft(loft, True, False)
        lipWire.translate(Base.Vector(0, 0, cycloid_length + 2 * lip))
        offset_cycloid_wire.translate(Base.Vector(0, 0, cycloid_length))
        loft2 = [lipWire, offset_cycloid_wire]
        loft2 = Part.makeLoft(loft2, True, False)
        result = result.fuse(loft).fuse(loft2).fuse(lipExtrude1).fuse(lipExtrude2)
    return result


# This builds the contracted cycloid and contra-cycloid with the central holes, eccentric and central axis hole.
def contracted_cycloid(ecc=0.0, z=0.0, rotate=0.0):
    cc = contracted_cycloid_blank()
    ainc = 360 / holes
    for r in range(holes):
        roller = roller_hole().translate(Base.Vector(dd / 2, 0, 0))
        roller.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), r * ainc + rotate)
        cc = cc.cut(roller)
    centre = Part.makeCylinder(dc / 2, cycloid_length * 2 + 2, Base.Vector(0, 0, -1), Base.Vector(0, 0, 1))
    cc = cc.cut(centre)
    cc = cc.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), rotate)
    cc.translate(Base.Vector(ecc, 0, z))
    return cc


def eccentric(ecc_diameter=10.0, bearing_length=5.0, z=0.0):
    ecc = Part.makeCylinder(ecc_diameter, bearing_length + 0.01, Base.Vector(0, 0, -1), Base.Vector(0, 0, 1))
    ecc.translate(Base.Vector(-e, 0, 1))
    ecc2 = Part.makeCylinder(ecc_diameter, bearing_length + 0.01, Base.Vector(0, 0, -1), Base.Vector(0, 0, 1))
    ecc2.translate(Base.Vector(-e, 0, z + 1))
    ecc2 = ecc2.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), 180)
    axis = Part.makeCylinder(central_axis_radius, bearing_length * 2 * 1.1 + 10, Base.Vector(0, 0, -10),
                             Base.Vector(0, 0, 10))
    d_shaft_cut = Part.makeBox(central_axis_radius * 2, central_axis_radius * 2, bearing_length * 2 * 1.11 + 10,
                               Base.Vector(-central_axis_radius, -central_axis_radius, -10), Base.Vector(0, 0, 10))
    d_shaft_cut.translate(Base.Vector(-d_shaft + central_axis_radius * 2, 0, 0))
    axis = axis.cut(d_shaft_cut)
    ecc = ecc.fuse(ecc2)
    ecc.removeSplitter()
    ecc = ecc.cut(axis)
    return ecc


def roller_disc(count=3, inner_roller_diameter=30.0, bearing_length=7.0, cycloid_length=2.5, stop_length=2.0):
    inner_roller = Part.makeCylinder(inner_roller_diameter / 2,
                                     bearing_length + stop_length,
                                     Base.Vector(0, 0, 0),
                                     Base.Vector(0, 0, 1))
    stop_ring = Part.makeCylinder(inner_roller_diameter / 2 + stop_length / 2,
                                  stop_length,
                                  Base.Vector(0, 0, 0),
                                  Base.Vector(0, 0, 1))
    inner_roller = inner_roller.fuse(stop_ring)
    for r in range(count):
        roller = Part.makeCylinder(dr / 2, bearing_length * 2 + 0.2, Base.Vector(dd / 2, 0, -0.1),
                                   Base.Vector(0, 0, 1.1))
        roller.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), r * 360 / count)
        pin_axis = Part.makeCylinder(r_pin_axis, bearing_length * 2 + 0.2, Base.Vector(dd / 2, 0, -0.1),
                                     Base.Vector(0, 0, 1.1))
        pin_axis.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), r * 360 / count)
        pin_base = Part.makeCylinder(r_pin_axis + 0.5, 0.2, Base.Vector(dd / 2, 0, -0.1), Base.Vector(0, 0, 1.1))
        pin_base.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), r * 360 / count)
        inner_roller = inner_roller.fuse(pin_base)
        inner_roller = inner_roller.cut(pin_axis)
    centre = Part.makeCylinder(dr / 2, cycloid_length * 2 + stop_length, Base.Vector(0, 0, -1), Base.Vector(0, 0, 1))
    inner_roller = inner_roller.cut(centre)
    inner_roller.translate(Base.Vector(0, 0, cycloid_length * 2 + 0.1))
    inner_roller = inner_roller.cut(Part.makeCylinder(3 / 2,
                                                      stop_length,
                                                      Base.Vector(0, 0, 0),
                                                      Base.Vector(0, 0, 1)))
    inner_roller.removeSplitter()
    return inner_roller


def get_bearings(count=10, diameter=45.0, external_radius=3.0, internal_radius=1.5, bearing_length=3.0):
    """
    This adds the pins, rollers to the model if required
    :param external_radius:
    :param diameter:
    :param count: number of bearings
    :param internal_radius: axis radius
    :param bearing_length:
    :return: Part bearings
    """
    pins = get_pins(count, diameter, internal_radius, bearing_length * 2 + 0.1)
    bearings = get_pins(count, diameter, external_radius, bearing_length * 2 + 0.1)
    bearings = bearings.cut(pins)
    return bearings


def get_pins(count=10, diameter=45.0, radius=1.5, length=3.0):
    pins = null_set()
    for r in range(count):
        pin = Part.makeCylinder(radius, length, Base.Vector(diameter / 2, 0, -0.1), Base.Vector(0, 0, 1.1))
        pin.rotate(Base.Vector(0, 0, 0), Base.Vector(0, 0, 1), r * 360 / count)
        pins = pins.fuse(pin)
    return pins


def ring_base(external_diameter=45.0, ring_length=6.0, bearing_diameter=42.0, bearing_length=7.0, eccentricity=2.0):
    base = Part.makeCylinder(external_diameter / 2,
                             bearing_length,
                             Base.Vector(0, 0, 0),
                             Base.Vector(0, 0, 1))
    base = base.cut(Part.makeCylinder(external_diameter / 2 - (external_diameter - bearing_diameter) / 2,
                                      bearing_length,
                                      Base.Vector(0, 0, 0),
                                      Base.Vector(0, 0, 1)))
    base = base.translate(Base.Vector(0, 0, ring_length))
    r = Part.makeCylinder(external_diameter / 2,
                          ring_length,
                          Base.Vector(0, 0, 0),
                          Base.Vector(0, 0, 1))
    r = r.cut(
        Part.makeCylinder(external_diameter / 2 - (external_diameter - bearing_diameter) / 2 + eccentricity,
                          ring_length,
                          Base.Vector(0, 0, 0),
                          Base.Vector(0, 0, 1)))
    r = r.fuse(base)
    r.removeSplitter()
    return r


def ring(count=10,
         pin_external_diameter=6.0,
         pin_internal_diameter=3.0,
         pin_bearing_length=2.5,
         external_diameter=45.0,
         bearing_diameter=42.0,
         bearing_length=7.0,
         inner_roller_diameter=30.0,
         stop_length=2.0,
         eccentricity=2.0):
    stop = Part.makeCylinder(external_diameter / 2 + pin_external_diameter / 2 + 1,
                             stop_length,
                             Base.Vector(0, 0, 0),
                             Base.Vector(0, 0, 1)) \
        .cut(Part.makeCylinder(inner_roller_diameter / 2 + 1,
                               stop_length,
                               Base.Vector(0, 0, 0),
                               Base.Vector(0, 0, 1)))
    stop.translate(Base.Vector(0, 0, pin_bearing_length * 2 + bearing_length))
    r = ring_base(external_diameter + pin_external_diameter + 2, pin_bearing_length * 2, bearing_diameter,
                  bearing_length,
                  eccentricity)
    pins = get_pins(count, external_diameter, pin_internal_diameter / 2, 1000)
    bearings = get_pins(count, external_diameter, pin_external_diameter / 2, pin_bearing_length * 2)
    r = r.fuse(stop)
    r = r.cut(pins)
    r = r.cut(bearings)
    r.removeSplitter()
    return r


def cover(count=10,
          pin_external_diameter=6.0,
          pin_internal_diameter=3.0,
          length=3.0,
          diameter=45.0):
    c = Part.makeCylinder(diameter / 2 + pin_external_diameter / 2 + 1,
                          length,
                          Base.Vector(0, 0, 0),
                          Base.Vector(0, 0, 1))
    pins = get_pins(count, diameter, pin_internal_diameter / 2, 1000)
    pins.translate(Base.Vector(0, 0, -length))
    c = c.cut(pins)
    c.translate(Base.Vector(0, 0, -length))
    return c


# Make the parts and show them
Part.show(eccentric(eccentric_diameter, cycloid_length, cycloid_length))
Part.show(contracted_cycloid(-e, 0, 0))
Part.show(contracted_cycloid(e, cycloid_length, 180.0))
Part.show(get_bearings(10, D, dp / 2, r_pin_axis, cycloid_length))
Part.show(roller_disc(3, 30.0, ring_bearing_length, cycloid_length, bearing_stop_length))
Part.show(get_bearings(3, dd, dr / 2, r_pin_axis, cycloid_length))

# Part.show(ring(N, D + dp + 4, 42.0, bearingLength * 2, 10, ring_bearing_length, 5))
Part.show(ring(N,
               small_bearing_external_diameter,
               small_bearing_internal_diameter,
               small_bearing_length))
Part.show(cover())
# ring_base(45.0, bearingLength * 2, ring_bearing_diameter, ring_bearing_length)

App.ActiveDocument.getObject("Shape").Label = "Eccentric"
App.ActiveDocument.getObject("Shape001").Label = "Cycloid"
App.ActiveDocument.getObject("Shape002").Label = "ContrCycloid"
App.ActiveDocument.getObject("Shape003").Label = "ExternalBearers"
App.ActiveDocument.getObject("Shape004").Label = "RollerDisc1"
App.ActiveDocument.getObject("Shape005").Label = "InternalBearers"
App.ActiveDocument.getObject("Shape006").Label = "Ring1"
