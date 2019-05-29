#!usr/bin/env python

from scipy.interpolate import splprep, splev
import numpy as np

#not needed for actual controller
import matplotlib.pyplot as plt



class pathTrackController:

	#defines the path
	waypoints = None  #list given of their value in the spline param
	spline = None

	closest_point = None

	x = None
	y = None
	theta = None

	curvature = None
	lateral_err = None
	heading_err = None

	search_param = 0.005

	control_rate = 50

	def __init__(self, control_params = None, search_param = None, control_rate = None):
		if control_params:
			self.v = control_params.get('v')
			self.k_lat = control_params.get('k_lat')
			self.k_head = control_params.get('k_head')
			self.max_omega = control_params.get('max_omega')
		else:
			self.v = 0.5
			self.k_lat = 0.25
			self.k_head = 0.75
			self.max_omega = 0.5

		if search_param:
			if search_param < 0 or search_param > 1:
				print 'search parameter must be between (0,1) exclusive, setting to default of 0.5cm'
			else:
				self.search_param = search_param


		if control_rate:
			assert(control_rate > 0), 'controller rate must be greater than 0'
			self.control_rate = control_rate



	@staticmethod
	def getDistance(x, y, s, spline):
		'''helper to get distance between an (x,y)and a point on the spline defined by s,
		where s is the normalized spline parameter from [0,1]'''
		point_on_u = splev(s, spline)
		return np.sqrt((x - point_on_u[0])**2 + (y - point_on_u[1])**2)



	@staticmethod
	def getCurvature(s, spline):
		'''gets the curvature of a point on the spline defined by its spline parameter value s'''
		first_der = splev(s, spline, der=1)
		second_der = splev(s, spline, der=2)

		x_d = first_der[0]
		y_d = first_der[1]
		x_dd = second_der[0]
		y_dd = second_der[1]

		num = x_d*y_dd - y_d*x_dd
		den = (x_d**2 + y_d**2)**1.5

		return np.true_divide(num,den)



	@staticmethod
	def unwrapPiSubtract(x, y):
		'''Subtract to find the heading error, making sure the heading error is oriented correctly
		and is within -pi to pi'''

		#if both in same hemisphere, then their difference would never exceed pi
		if np.sign(x) == np.sign(y):
			return x - y

		mag_diff = abs(x-y)

		if mag_diff <= np.pi:
			return x - y
		else:
			if np.sign(x) == 1 and np.sign(y) == -1:
				return -(2*np.pi - mag_diff)
			else: 
				return 2*np.pi - mag_diff



	def nearestWaypoint(self):
		''' find the nearest waypoint to (x,y), where waypoints are the points used to fit the spline
		u is a list of the waypoints expressed in terms of the spline parameter'''
		closest_point = 0
		best_distance = np.inf

		for s in self.waypoints:
			d = self.getDistance(self.x, self.y, s, self.spline)

			if d < best_distance:
				best_distance = d
				closest_point = s

		self.closest_point = closest_point #returns closest_point as a spline parameter




	def nearestOnSpline(self):
		'''given the last nearest point on the spline, get the updates the newest nearest point
		Note that it could be the same as the last nearest point'''

		ds = self.search_param
		last_nearest = self.closest_point

		#have to recompute nearest distance from current location to the last nearest
		prev_nearest_dist = self.getDistance(self.x, self.y, last_nearest, self.spline)  
		forward = self.getDistance(self.x, self.y, min(last_nearest+ds,1), self.spline)
		backward = self.getDistance(self.x, self.y, max(last_nearest-ds,0), self.spline)


		if prev_nearest_dist < forward and prev_nearest_dist <= backward:
			#nearest point hasn't changed
			self.closest_point = last_nearest
			return 

		elif forward < backward:
			#nearest point is further ahead on the spline'
			closest_point = min(last_nearest + ds,1)
			best_distance = forward

			while min(closest_point+ds, 1) <= 1:
				new_point = min(closest_point+ds, 1)
				new_dist = self.getDistance(self.x, self.y, new_point, self.spline)
				if new_dist >= best_distance:
					self.closest_point = closest_point
					return 
				else:
					closest_point = new_point
					best_distance = new_dist


		elif backward < forward: 
			#nearest point is further back on the spline
			closest_point = max(last_nearest - ds,0)
			best_distance = backward

			while max(closest_point-ds, 0) >= 0:
				new_point = max(closest_point-ds, 0)
				new_dist = self.getDistance(self.x, self.y, new_point, self.spline)
				if new_dist >= best_distance:
					self.closest_point = closest_point
					return 
				else:
					closest_point = new_point
					best_distance = new_dist

    


	def setPath(self, path_points_x, path_points_y):
		'''called by the main to set a new path'''
		if self.x == None or self.y == None or self.theta == None:
			print 'cannot set paths until state is initialized'
			return
		else:
			assert (path_points_x.shape == path_points_y.shape), 'x and y point array shape mismatch'

			self.spline, self.waypoints = splprep([path_points_x, path_points_y], s=0)
 
			self.nearestWaypoint()
			self.nearestOnSpline()

			print self.closest_point

			self.curvature = self.getCurvature(self.closest_point, self.spline)

			print self.curvature


	def trackingErrors(self):
		'''updates the lateral and heading (in radians) errors'''

		closest_point_coordinates = splev(self.closest_point, self.spline)
		x_ref = closest_point_coordinates[0]
		y_ref = closest_point_coordinates[1]

		dx = self.x - x_ref
		dy = self.y - y_ref

		#spline unit tangent at closest point
		tangent = splev(self.closest_point, self.spline, der=1)
		norm_tangent = np.sqrt(tangent[0]**2 + tangent[1]**2)
		tx = np.true_divide(tangent[0], norm_tangent)
		ty = np.true_divide(tangent[1], norm_tangent)

		#set to 0 if norm is 0, prevent NaN from divide by zero
		if np.isnan(tx):
			tx = 0
		if np.isnan(ty):
			ty = 0

		self.lateral_err = dx*ty - dy*tx
		self.heading_err = self.unwrapPiSubtract(theta, np.arctan2(ty,tx))




	@staticmethod
	def controlOmega(v, lateral_err, heading_err, curve, k_lat, k_head):
		'''control law from "A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles"'''

		#curvature following term
		first_term = np.true_divide(v*curve*np.cos(heading_err), (1 - curve*lateral_err))

		#heading error term
		second_term = k_head * abs(v) * heading_err

		#lateral error term
		third_term = np.true_divide(k_lat * v * np.sin(heading_err), heading_err) * lateral_err


		#set to zero in case first term is NaN from divide by zero
		if np.isnan(first_term):
			first_term = 0

		#set to zero in case second term is NaN from divide by zero
		if np.isnan(third_term):
			third_term = 0


		return first_term - second_term + third_term  #third term should be a plus sign (i think)




	def updateControlStates(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta


	def calcControlOutput(self, x_est, y_est, theta_est):
		#update controller state estimate from the estimator
		self.updateControlStates(x_est, y_est, theta_est)

		self.nearestOnSpline()
		self.curvature = self.getCurvature(self.closest_point, self.spline)


		self.trackingErrors()

		v = self.v
		k_lat = self.k_lat
		k_head = self.k_head
		omega = self.controlOmega(v, self.lateral_err, self.heading_err, self.curvature, k_lat, k_head)

		if abs(omega) > self.max_omega:
			omega = np.sign(omega) * self.max_omega

		return v, omega




if __name__ == '__main__':

	control_params = {}
	control_params['v'] = 0.5

	control_params['k_lat'] = 0.25
	control_params['k_head'] = 0.75

	control_params['max_omega'] = 0.4
  

	controller = pathTrackController(control_params)


	phi = np.linspace(0, 0.9*np.pi, 40)
	r = 0.5 + np.cos(phi)         # polar coords
	xp, yp = r * np.cos(phi), r * np.sin(phi)    # convert to cartesian
	xp = xp*10
	yp = yp*10


	x_i = 12
	y_i = 0
	theta_i = 90 * np.pi / 180

	controller.updateControlStates(x_i, y_i, theta_i)

	controller.setPath(xp, yp)

	new_points = splev(controller.waypoints, controller.spline)

	fig, ax = plt.subplots()
	ax.plot(xp, yp, 'ro')
	ax.plot(new_points[0], new_points[1], 'r-')
	ax.plot(x_i, y_i, 'bo')
	ax.axis('equal')
	plt.show()

	#initialize a list to track pose over the trajectory
	init_pose = [x_i,y_i,theta_i]
	poses = []
	poses.append(init_pose)


	timestep = 0.01
	end_time = 60
	current_time = 0


	while current_time <= end_time:
		x = poses[-1][0]
		y = poses[-1][1]
		theta = poses[-1][2]

		v, omega = controller.calcControlOutput(x, y, theta)

		# compute x_dot and y_dot and theta_dot and the next time step pose
		x_dot = v * np.cos(theta)
		y_dot = v * np.sin(theta)
		theta_dot = omega

		x_next = x + x_dot * timestep
		y_next = y + y_dot * timestep
		theta_next = theta + theta_dot * timestep

		#need to wrap theta within -pi to pi
		if theta_next > np.pi:
		    theta_next = theta_next - 2*np.pi
		    
		elif theta_next < -np.pi:
		    theta_next = theta_next + 2*np.pi


		#append next time step pose to the end of pose and increment current_time by timestep
		poses.append([x_next, y_next, theta_next])
		current_time += timestep   


	x_c = np.zeros([len(poses)])
	y_c = np.zeros([len(poses)])

	for i in xrange(len(poses)):
	    x_c[i] = poses[i][0]
	    y_c[i] = poses[i][1]
	    


	fig, ax = plt.subplots(figsize=(12,10))
	ax.plot(xp, yp, 'ro')
	ax.plot(new_points[0], new_points[1], 'r-')
	ax.plot(x_c, y_c, 'c-')
	ax.axis('equal')
	plt.show()


