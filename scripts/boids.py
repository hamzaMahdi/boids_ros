#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from util import Vector2, angle_diff, MAFilter


# the three functions below interface the standard ROS messages with the Vector2 type
def get_agent_velocity(agent):
    """Return agent velocity as Vector2 instance."""
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel


def get_agent_position(agent):
    """Return agent position as Vector2 instance."""
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos


def get_obst_position(obst):
    """Return obstacle position as Vector2 instance."""
    pos = Vector2()
    pos.x = obst.position.x
    pos.y = obst.position.y
    return pos


class Boid(object):

    def __init__(self, initial_velocity_x, initial_velocity_y, wait_count, start_count, frequency):
        """Create an empty boid and update parameters."""
        self.position = Vector2()
        self.velocity = Vector2()
        self.mass = 0.18  # Mass of Sphero robot in kilograms
        self.wait_count = wait_count  # Waiting time before starting
        self.start_count = start_count  # Time during which initial velocity is being sent
        self.frequency = frequency  # Control loop frequency
        # make sure that if the object is already in collision, it does not count again
        self.prev_collision = False
        self.prev_wall_collision = False
        # Set initial velocity
        self.initial_velocity = Twist()
        self.initial_velocity.linear.x = initial_velocity_x
        self.initial_velocity.linear.y = initial_velocity_y

        # This dictionary holds values of each flocking components and is used
        # to pass them to the visualization markers publisher.
        self.viz_components = {}

    def update_parameters(self, params):
        """Save Reynolds controller parameters in class variables."""
        self.alignment_factor = params['alignment_factor']
        self.cohesion_factor = params['cohesion_factor']
        self.separation_factor = params['separation_factor']
        self.avoid_factor = params['avoid_factor']
        self.max_speed = params['max_speed']
        self.max_force = params['max_force']
        self.crowd_radius = params['crowd_radius']
        self.search_radius = params['search_radius']
        self.avoid_radius = params['avoid_radius']
        self.avoid_kp = params['avoid_kp']

        # Scaling is calculated so that force is maximal when agent is
        # 0.85 * search_radius away from obstacle. Formula was obtained from https://github.com/mkrizmancic
        self.avoid_scaling = 1 / ((0.85 * self.search_radius) ** 2 * self.max_force)

        # Scaling is calculated so that cohesion and separation forces
        # are equal when agents are crowd_radius apart. Formula was obtained from https://github.com/mkrizmancic
        self.separation_gain = self.search_radius / self.crowd_radius ** 3 / self.max_force

        rospy.loginfo(rospy.get_caller_id() + " -> Parameters updated")
        rospy.logdebug('alignment_factor:  %s', self.alignment_factor)
        rospy.logdebug('cohesion_factor:  %s', self.cohesion_factor)
        rospy.logdebug('separation_factor:  %s', self.separation_factor)
        rospy.logdebug('avoid_factor:  %s', self.avoid_factor)
        rospy.logdebug('max_speed:  %s', self.max_speed)
        rospy.logdebug('max_force:  %s', self.max_force)
        rospy.logdebug('crowd_radius:  %s', self.crowd_radius)
        rospy.logdebug('search_radius:  %s', self.search_radius)
        rospy.logdebug('avoid_radius:  %s', self.avoid_radius)


    def check_collisions(self, nearest_agents,obstacles):
        sphero_radius = 0.036
        collision = False
        wall_collision = False
        # Find if nearby agents are too close
        for agent in nearest_agents:
            # set the minimum distance before counting a collision
            if get_agent_position(agent).norm()<=sphero_radius*2.3: 
                print("collision detected")
                collision = True

        for obstacle in obstacles:
            # set the minimum distance before counting a collision
            if get_obst_position(obstacle).norm()<=sphero_radius*1.15: 
                print("wall collision detected")
                wall_collision = True
        # save the current collision state
        if(collision and not self.prev_collision):
            self.prev_collision = True
        else:
            collision = False
        if(wall_collision and not self.prev_wall_collision):
            self.prev_wall_collision = True
        else:
            wall_collision = False
        if(not collision):
            self.prev_collision = False
        if(not wall_collision):
            self.prev_wall_collision = False
        return collision, wall_collision

    def rule1_cohesion(self, nearest_agents):
        mean_position = Vector2()
        direction = Vector2()
        # Find mean position of neighboring agents.
        for agent in nearest_agents:
            mean_position += get_agent_position(agent)

        # average but dont divide by zero.
        if nearest_agents:
            direction = mean_position / len(nearest_agents)
            rospy.logdebug("cohesion*:    %s", direction)
            # limit the force to max if necessary
            direction.set_mag((self.max_force * (direction.norm() / self.search_radius)))
            if direction.norm() > self.max_force:
                direction.set_mag(self.max_force)
        return direction

    def rule2_separation(self, nearest_agents):
        direction = Vector2()
        count = 0

        # Calculate repulsive force for each neighboring agent in sight.
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            d = agent_position.norm()
            # we only want to calculate if the agents are close to each other
            if d < self.crowd_radius:
                count += 1
                agent_position *= -1  # Make vector point away from other agent.
                agent_position.normalize()  # Normalize to get only direction.
                # Vector's magnitude is proportional to inverse square of the distance between agents.
                # this formula was obtained from https://github.com/mkrizmancic
                agent_position = agent_position / (self.separation_gain * d ** 2)
                direction += agent_position
        # average but dont divide by 0 (if agents are sufficiently far from each other)
        if count:
            direction /= count
            # 3 * max force because priority is given to obstacle avoidance as suggested Conrad Parker
            if direction.norm() > 3 * self.max_force:
                direction.set_mag(3 * self.max_force)
        rospy.logdebug("separation*:  %s", direction)
        return direction

    def rule3_alignment(self, nearest_agents):
        mean_velocity = Vector2()
        steer = Vector2()
        # Find mean direction of neighboring agents.
        for agent in nearest_agents:
            mean_velocity += get_agent_velocity(agent)
        rospy.logdebug("alignment*:   %s", mean_velocity)
        if nearest_agents:
            mean_velocity.set_mag(self.max_speed)
            steer = mean_velocity - self.velocity
            if steer.norm() > self.max_force:
                steer.set_mag(self.max_force)
        return steer

    def braitenberg(self, obstacles):
        """
        obstacle avoidance based on simple force field representation of obstacle
        """
        main_direction = Vector2()
        safety_direction = Vector2()
        count = 0

        # Calculate repulsive force for each obstacle in sight.
        for obst in obstacles:
            obst_position = get_obst_position(obst)
            d = obst_position.norm()
            obst_position *= -1  # Make vector point away from obstacle.
            obst_position.normalize()  # Normalize to get only direction.
            # If obstacle is very close
            if d < self.avoid_radius:
                safety_direction += d * self.avoid_kp * obst_position
                count += 1

            # For all other obstacles: scale with inverse square law.
            # this formula was obtained from https://github.com/mkrizmancic
            obst_position = obst_position / (self.avoid_scaling * d ** 2)
            main_direction += obst_position

        if obstacles:
            # this calculates the average and makes sure the turning force is not too extreme,
            # as well as turning the avoidance based on simple proportionality factor
            main_direction = main_direction / len(obstacles) * self.avoid_kp * \
                             max(math.cos(math.radians(angle_diff(self.old_heading, main_direction.arg() + 180))), 0)

        if count:
            safety_direction /= count

        rospy.logdebug("avoids*:      %s", main_direction)

        return main_direction + safety_direction

    def follow_heading(self, angle, target):
        '''compute rotational vel based on heading'''
        kp = 0.1
        return (target - angle) * kp

    def orient(self, force, target):
        '''compute linear vel based on heading'''
        a = force.x - target
        side_scaling = a * -0.05
        main_direction = side_scaling
        return main_direction

    def move_agent_to_new_position(self, my_agent, nearest_agents, obstacles):
        """Compute total velocity based on all components."""

        # While waiting to start, send zero velocity and decrease counter.
        if self.wait_count > 0:
            self.wait_count -= 1
            return Twist(), None, [False, False]

        # Send initial velocity and decrease counter.
        elif self.start_count > 0:
            self.start_count -= 1
            return self.initial_velocity, None, [False, False]

        # Normal operation, velocity is determined using Reynolds' rules.
        else:
            self.velocity = get_agent_velocity(my_agent)
            self.old_heading = self.velocity.arg()
            self.old_velocity = Vector2(self.velocity.x, self.velocity.y)

            # detect collisions
            collision_vector = self.check_collisions(nearest_agents,obstacles)
            
            # calculate force fields
            cohesion = self.rule1_cohesion(nearest_agents)
            separation = self.rule2_separation(nearest_agents)
            alignment = self.rule3_alignment(nearest_agents)
            avoid = self.braitenberg(obstacles)
            # Add components together and limit the output.
            force = Vector2()
            force += alignment * self.alignment_factor
            force += cohesion * self.cohesion_factor
            force += separation * self.separation_factor
            force += avoid * self.avoid_factor
            force.x+= self.orient(force,1) #go right
            force.limit(self.max_force)
            # those are not necessary (can be made into a single constant) but nice since they contain actual formualae
            acceleration = force / self.mass
            self.velocity += acceleration / self.frequency
            self.velocity.limit(self.max_speed)

            # Return the the velocity as Twist message.
            vel = Twist()
            vel.linear.x = self.velocity.x
            vel.linear.y = self.velocity.y

            # Pack all components for Rviz visualization.
            # Make sure these keys are the same as the ones in `util.py`.
            self.viz_components['alignment'] = alignment * self.alignment_factor
            self.viz_components['cohesion'] = cohesion * self.cohesion_factor
            self.viz_components['separation'] = separation * self.separation_factor
            self.viz_components['avoid'] = avoid * self.avoid_factor
            self.viz_components['acceleration'] = acceleration / self.frequency
            self.viz_components['velocity'] = self.velocity
            self.viz_components['estimated'] = self.old_velocity
            return vel, self.viz_components, collision_vector
