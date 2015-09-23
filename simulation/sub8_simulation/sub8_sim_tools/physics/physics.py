#!/usr/bin/env python
'''Physics components for the Sub8 simulator, Simul8
Original Author: Annie Luc
'''
from __future__ import division
import traceback
import numpy as np
import time
import sys
import os
import random
import time
import ode


class Constants(object):
    density_water = 999.97  # kg / m**3
    g = -9.81  # m / s**2, gravitational constant


class World(object):
    def __init__(self, dt):
        '''This section incorporates PyODE to define methods to create and draw three objects: a sphere, box, or cylinder.
        Some code borrowed from http://sourceforge.net/p/pyode/mailman/message/19674697/.
        '''
        self.dt = dt
        self.entities = []
        # Create a world object
        self.ode_world = ode.World()
        self.ode_world.setGravity(np.array([0, 0, Constants.g]))    

        self.ode_world.setERP(0.8)
        self.ode_world.setCFM(1E-5)
    
        # Create a space object
        self.space = ode.Space()
    
        # Create a plane geom which prevent the objects from falling forever
        self.floor = ode.GeomPlane(self.space, (0., 0., 1.), -5.)
        self.contact_group = ode.JointGroup()

    def add_entity(self, Entity_Type, *args, **kwargs):
        '''Add an arbitrary entity
        Example:
            Say we want to add a Sphere to my physics world
            from physics import Sphere, World
            w = World(dt=0.1)
            w.add_entity(Sphere, postion, density, radius)

        NOTE that we did not instantiate a Sphere, there was no Sphere(~~~)
        We just passed the Sphere class, not a Sphere instance
        This is for greater extensibility without repeated code
        '''
        entity = Entity_Type(self.ode_world, self.space, *args, **kwargs)
        self.entities.append(entity)
        return entity

    def step(self, dt=None):
        if dt is None:
            dt = self.dt

        self.space.collide((self.ode_world, self.contact_group), self.near_callback)
        self.ode_world.step(dt / 2)
        for entity in self.entities:
            entity.step(dt / 2)

        # Remove all contact joints
        self.contact_group.empty()

    def near_callback(self, (world, contact_group), geom1, geom2):
        '''Callback function for the collide() method.
        This function checks if the given geoms do collide and
        creates contact joints if they do.
        '''
        # Check if the objects do collide
        contacts = ode.collide(geom1, geom2)
        # Create contact joints
        for c in contacts:
            c.setBounce(.4)
            c.setMu(5000)
            j = ode.ContactJoint(world, contact_group, c)
            j.attach(geom1.getBody(), geom2.getBody())


class Entity(object):
    _linear_damping_coeff = -0.2
    _rotational_damping_coeff = -0.2

    def __init__(self, world, space):
        self.body = ode.Body(world)

    def step(self):
        raise(NotImplementedError("You must implement a step function!"))

    @property
    def submerged_volume(self):
        '''Currently, no depth input
        Note that without some depth, a positively buoyant object will fly away forever
        '''
        raise(NotImplementedError("You must implement a submerged volume getter!"))

    @property
    def pos(self):
        return np.array(self.body.getPosition(), dtype=np.float32)

    @property
    def pose(self):
        '''Construct a 4x4 homogeneous pose matrix,
        Pose changes frequently enough that we recompute each time
        '''
        pose = np.zeros((4, 4))
        orientation = np.array(self.body.getRotation(), dtype=np.float32)
        orientation = orientation.reshape(3, 3)
        # Don't use self.pos, to avoid a wasteful array cast
        position = self.body.getPosition()
        pose[:3, :3] = orientation
        pose[3, :3] = position
        pose[3, 3] = 1.
        return pose

    @property
    def velocity(self):
        linear_vel = np.array(self.body.getLinearVel(), dtype=np.float32)
        angular_vel = np.array(self.body.getAngularVel(), dtype=np.float32)
        return linear_vel, angular_vel

    def apply_buoyancy_force(self):
        '''Apply buoyancy force based on submerged volume approximation
        '''
        submerged_volume = self.submerged_volume
        # Negative, because g < 0
        buoyancy_force = -Constants.density_water * Constants.g * submerged_volume
        self.body.addForce((0.0, 0.0, buoyancy_force / 100.))

    def apply_damping_force(self):
        '''Apply a linear damping force'''
        velocity = np.array(self.body.getLinearVel(), dtype=np.float32)
        norm_velocity = self._linear_damping_coeff * (np.linalg.norm(velocity) ** 2)
        self.body.addForce(norm_velocity * velocity)

    def apply_damping_torque(self):
        '''Apply a linear rotational damping torque'''
        angular_velocity = np.array(self.body.getAngularVel(), dtype=np.float32)
        norm_velocity = self._rotational_damping_coeff * (np.linalg.norm(angular_velocity) ** 2)
        self.body.addTorque(norm_velocity * angular_velocity)


class Box(Entity):
    _linear_damping_coeff = -0.3  # TODO: Estimate area
    _rotational_damping_coeff = -0.5  # TODO: Estimate area

    def __init__(self, world, space, position, density, lx, ly, lz):
        self.body = ode.Body(world)
        self.body.setPosition(position)

        M = ode.Mass()
        M.setBox(density, lx, ly, lz)
        self.body.setMass(M)

        self.geom = ode.GeomBox(space, lengths=(lx, ly, lz))
        self.geom.setBody(self.body)

    def step(self, dt):
        # TODO: Improve from spherical approximation
        self.apply_damping_force()
        self.apply_damping_torque()


class Sphere(Entity):
    _linear_damping_coeff = -0.4
    _rotational_damping_coeff = -0.05

    def __init__(self, world, space, position, density, radius):
        self.body = ode.Body(world)
        self.body.setPosition(position)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)
        self.radius = radius

        # Create a Sphere geom for collision detection
        self.geom = ode.GeomSphere(space, radius)
        self.geom.setBody(self.body)

    @property
    def submerged_volume(self):
        '''Assume water is at z = 0
        Volume of sphere: (4 / 3)pi * r**3
        Volume of a spherical cap of height h: (1 / 3)pi * h**2 * (3r - h)
        '''
        h = np.clip(self.pos[2], 0.0, 2 * self.radius)
        sphere_volume = (4. / 3.) * (np.pi * (self.radius ** 3.))
        above_water_volume = (1 / 3) * np.pi * (h ** 2) * ((3 * self.radius) - h)
        submerged_volume = sphere_volume - above_water_volume
        return submerged_volume

    def step(self, dt):
        self.apply_damping_force()
        self.apply_damping_torque()
        self.apply_buoyancy_force()


class Mesh(Entity):
    def __init__(self, pos, density, radius, height, color):
        '''Class to represent a Mesh object. 
           For now the object follows the physics of a sphere object in PyODE.
        '''
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)

        # Create a Sphere geom for collision detection
        self.geom = ode.GeomSphere(space, radius)
        self.geom.setBody(self.body)

    def step(self, dt):
        self.apply_damping_force()
        self.apply_damping_torque()