(in-package :rgn-geo)

(defmethod centroidish ((self direct::2d-line))
  (let ((mid (the direct::mid)))
    (vec3 (vx mid) (vy mid) 0.0d0)))

(defmethod centroidish ((self direct::circle))
  (let ((cen (the direct::center)))
    (vec3 (vx cen) (vy cen) 0.0d0)))

(defmethod centroidish ((self direct::ellipse))
  (let ((cen (the direct::center)))
    (vec3 (vx cen) (vy cen) 0.0d0)))

(defmethod centroidish ((self direct::line))
  (the direct::mid))

(defmethod centroidish ((self direct::arc))
  (let* ((c (the direct::center))
	 (val (v* (the direct::radius) (vec3 (- (sin (the direct::end-angle)) (sin (the direct::start-angle)))
				     (- (cos (the direct::start-angle)) (cos (the direct::end-angle)))
				     0.0d0))))
    (if (eq (the direct::orientation) :ccw)
	(v+ (vec3 (vx c) (vy c) 0.0d0) val)
	(v- (vec3 (vx c) (vy c) 0.0d0) val))))

(defmethod centroidish ((self direct::bezier-space-curve))
  (let ((hcpoints (the direct::homogeneous-control-points)))
    (loop for cp across (map 'vector #'safe-euclid hcpoints)
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum (vec3 (vx cp) (vy cp) (vz cp))))
	  finally (return (v/ sum (length hcpoints))))))

(defmethod centroidish ((self direct::bezier-plane-curve))
  (let ((hcpoints (the direct::homogeneous-control-points)))
    (loop for cp across (map 'vector #'(lambda (vec3)
					 (if (= (vz vec3) 0.0d0)
					     vec3
					     (vec3 (/ (vx vec3) (vz vec3)) (/ (vy vec3) (vz vec3)) 0.0d0)))
			     hcpoints)
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum cp))
	  finally (return (v/ sum (length hcpoints))))))

(defmethod centroidish ((self direct::b-spline-space-curve))
  (let ((hcpoints (the direct::homogeneous-control-points)))
    (loop for cp across (map 'vector #'safe-euclid hcpoints)
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum (vec3 (vx cp) (vy cp) (vz cp))))
	  finally (return (v/ sum (length hcpoints))))))

(defmethod centroidish ((self direct::b-spline-plane-curve))
  (let ((hcpoints (the direct::homogeneous-control-points)))
    (loop for cp across (map 'vector #'(lambda (vec3)
					 (if (= (vz vec3) 0.0d0)
					     vec3
					     (vec3 (/ (vx vec3) (vz vec3)) (/ (vy vec3) (vz vec3)) 0.0d0)))
			     hcpoints)
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum cp))
	  finally (return (v/ sum (length hcpoints))))))
  
