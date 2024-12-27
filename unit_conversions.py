import math

def inches_meters(inches):
    ''' Converts inches to meters '''
    return inches * 0.0254

def meters_inches(meters):
    ''' Converts meters to inches '''
    return meters / 0.0254

def feet_meters(feet):
    ''' Converts feet to meters '''
    return feet * 0.3048

def meters_feet(meters):
    ''' Converts meters to feet '''
    return meters / 0.3048

def degrees_radians(degrees):
    ''' Converts degrees to radians '''
    return degrees * (math.pi/180)

def radians_degrees(radians):
    ''' Converts radians to degrees '''
    return radians * (180/math.pi)

def rotations_degrees(rotations):
    ''' Converts rotations to degrees '''
    return rotations * 360  

def degrees_rotations(degrees):
    ''' Converts degrees to rotations '''
    return degrees / 360    

def rotations_radians(rotations):
    ''' Converts rotations to radians '''
    return rotations * 2 * math.pi

def radians_rotations(radians):
    ''' Converts radians to rotations '''
    return radians / (2 * math.pi)

def rotations_meters(rotations, wheel_diameter):
    ''' Converts rotations to meters '''
    return rotations * wheel_diameter * math.pi 

def meters_rotations(meters, wheel_diameter):
    ''' Converts meters to rotations '''
    return meters / (wheel_diameter * math.pi)

def rotations_inches(rotations, wheel_diameter):
    ''' Converts rotations to inches '''
    return meters_inches(rotations_meters(rotations, wheel_diameter))

def inches_rotations(inches, wheel_diameter):
    ''' Converts inches to rotations '''
    return meters_rotations(inches_meters(inches), wheel_diameter)

