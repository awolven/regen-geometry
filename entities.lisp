(adhoc:defpackage rgn-geo
  (:use :3dm :adhoc-scene-graph)
  (:shadow #:intersection)
  (:shadow #:locally)
  (:export #:intersection #:half #:twice)
  (:import-from :3dm.d
		#:vec2
		#:vec3
		#:vec4
		#:mat2
		#:mat3
		#:mat4
		#:matn
		#:quat
		#:meye
		#:with-vec2
		#:with-vec3
		#:with-vec4
		#:with-vec))
  

(in-package :rgn-geo)

(defun half (number)
  (/ number 2))

(defun twice (number)
  (* number 2))

(defmacro intersection (entity-a entity-b pick)
  `(intersection-1 ,entity-a ,entity-b ,pick ,self))

(defconstant +epsilon+ 1.0d-7)

(defun within-tolerance (a val &optional (tol +epsilon+))
  (<= (abs (- a val)) tol))

(defun d2r (d)
  (* (/ d 180.0d0) pi))

(defconstant 2pi #.(* pi 2.0d0))

(defconstant pi/2 #.(/ pi 2.0d0))

(defconstant pi/3 #.(/ pi 3.0d0))

(defconstant pi/4 #.(/ pi 4.0d0))

(defun factorial (n)
  (if (<= n 1)
      1
      (* n (factorial (1- n)))))

(defun gamma-m+1/2 (m+1/2)
  (let ((m (- m+1/2 1/2)))
    (labels ((foo (n)
	       (if (< n 1)
		   1
		   (* n (foo (- n 2))))))
      (* #.(sqrt pi) (/ (foo (1- (* 2 m))) (expt 2 m))))))

(defun binomial-coefficient-0.5 (k)
  (/ (gamma-m+1/2 1/2) (* (factorial k) (gamma-m+1/2 (- 1/2 k)))))

(defun binomial-coefficient (n k)
  (cond ((< n 0) (error "< n 0"))
	((or (> k n) (< k 0)) 0)
	((= k 0) 1)
	(t (+ (binomial-coefficient (1- n) (1- k))
	      (binomial-coefficient (1- n) k)))))

(defobject entity-mixin (selectable-mixin node-mixin)
  :attributes
  ((group :uncached (the superior group))

   (origin :uncached (if (the superior)
			 (the superior origin)
			 (vec3 0 0 0)))
   
   (rotation :uncached (if (the superior)
			   (the superior rotation)
			   (quat 0 0 0 1)))
   
   (rotation-matrix :uncached (meye 4))
   (translation-matrix :uncached (meye 4))
   (matrix :uncached (meye 4))
   (global-matrix :uncached (if (the superior)
				(the superior global-matrix)
				(the matrix)))
   (inverse-matrix :uncached (meye 4))
   (inverse-global-matrix :uncached (if (the superior)
					(the superior inverse-global-matrix)
					(the inverse-matrix)))					
   (node-children :uncached (fset:set))))

(defobject curve-mixin ())
(defobject line-mixin ())
(defobject plane-curve-mixin (curve-mixin))
(defobject space-curve-mixin (curve-mixin))

(defobject 2d-line-mixin (plane-curve-mixin line-mixin)
  :attributes
  ((degenerate? :uncached (v= (cl:the vec2 (the start)) (cl:the vec2 (the end))))
   (start       :uncached (v- (cl:the vec2 (the end)) (cl:the vec2 (the span))))
   (end         :uncached (v+ (cl:the vec2 (the start)) (cl:the vec2 (the span))))
   (span        :uncached (v- (cl:the vec2 (the end)) (cl:the vec2 (the start))))
   (mid         :uncached (v+ (cl:the vec2 (the start)) (v* 0.5d0 (the span))))
   (length      :uncached (vlength (cl:the vec2 (the span))))
   (direction   :uncached (if (the degenerate?)
			      (vec2 0 0)
			      (vunit (cl:the vec2 (the span)))))
   (domain      :uncached (list 0.0d0 1.0d0))
   (degree      :uncached 1)
   (closed?     :uncached nil)
   (periodic?   :uncached nil)))

(defmethod centroidish ((self 2d-line-mixin))
  (let ((mid (the mid)))
    (vec3 (vx mid) (vy mid) 0.0d0)))

(defmethod evaluate1 ((self 2d-line-mixin) (parameter real) &key (derivs 0))
  (let ((p (v+ (the start) (v* parameter (the span)))))
    (cond ((= derivs 1) (values p (the direction)))
	  ((= derivs 0) (values p))
	  ((> derivs 1) (values p (the direction) (vec2 0 0))))))

(defmethod to-opencascade ((self 2d-line-mixin))
  (let* ((start (the start))
	 (end   (the end))
	 (tail  (gp:pnt (vx start) (vy start) 0.0d0))
	 (head  (gp:pnt (vx end)   (vy end)   0.0d0)))
    (oc::value (make-instance 'oc::gc-make-segment
			      :P1 tail
			      :P2 head))))

(defun assure-double-float (value)
  (assert (typep value 'real))
  (coerce value 'double-float))

(defun assure-nonzero-double-float (value)
  (assert (typep value 'real))
  (assert (not (zerop value)))
  (coerce value 'double-float))

(defun assure-nonnegative-double-float (value)
  (assert (typep value '(real 0)))
  (coerce value 'double-float))

(defun assure-double-vec2 (value)
  (etypecase value
    (3dm.d::vec2 value)
    (3dm.f::vec2 (coerce-to-double-float value))
    (3dm.rat::vec2 (coerce-to-double-float value))))

(defun assure-double-vec3 (value)
  (etypecase value
    (3dm.d::vec3 value)
    (3dm.f::vec3 (coerce-to-double-float value))
    (3dm.rat::vec3 (coerce-to-double-float value))))

(defun assure-double-vec4 (value)
  (etypecase value
    (3dm.d::vec4 value)
    (3dm.f::vec4 (coerce-to-double-float value))
    (3dm.rat::vec4 (coerce-to-double-float value))))

(defclass 2d-line ()
  ()
  (:metaclass clui::protocol-class))

(defobject 2d-line_a (2d-line-mixin entity-mixin)
  :inputs
  ((start (:validator (assure-double-vec2 value)))
   (end   (:validator (assure-double-vec2 value)))))

(defobject 2d-line_b (2d-line-mixin entity-mixin)
  :inputs
  ((start (:validator (assure-double-vec2 value)))
   (mid   (:validator (assure-double-vec2 value))))

  :attributes
  ((span :uncached (v* 2.0d0 (v- (cl:the vec2 (the mid)) (cl:the vec2 (the start)))))))

(defobject 2d-line_c (2d-line-mixin entity-mixin)
  :inputs
  ((mid (:validator (assure-double-vec2 value)))
   (end (:validator (assure-double-vec2 value))))

  :attributes
  ((span :uncached (v* 2.0d0 (v- (cl:the vec2 (the end)) (cl:the vec2 (the mid)))))))

(defobject 2d-line_d (2d-line-mixin entity-mixin)
  :inputs
  ((start (:validator (assure-double-vec2 value)))
   (span  (:validator (assure-double-vec2 value)))))

(defobject 2d-line_e (2d-line-mixin entity-mixin)
  :inputs
  ((start     (:validator (assure-double-vec2 value)))
   (direction (:validator (assure-double-vec2 value)))
   (length    (:validator (assure-double-float value))))

  :attributes
  ((span :uncached (v* (cl:the double-float (the length)) (cl:the vec2 (the direction))))))

(defobject 2d-line_f (2d-line-mixin entity-mixin)
  :inputs
  ((end  (:validator (assure-double-vec2 value)))
   (span (:validator (assure-double-vec2 value)))))

(defobject 2d-line_g (2d-line-mixin entity-mixin)
  :inputs
  ((end       (:validator (assure-double-vec2 value)))
   (direction (:validator (assure-double-vec2 value)))
   (length    (:validator (assure-double-float value))))

  :attributes
  ((span :uncached (v* (cl:the double-float (the length)) (cl:the vec2 (the direction))))))

(defobject 2d-line_h (2d-line-mixin entity-mixin)
  :inputs
  ((mid  (:validator (assure-double-vec2 value)))
   (span (:validator (assure-double-vec2 value))))

  :attributes
  ((start :uncached (v- (cl:the vec2 (the mid)) (v* 0.5d0 (cl:the vec2 (the span)))))))

(defobject 2d-line_i (2d-line-mixin entity-mixin)
  :inputs
  ((mid       (:validator (assure-double-vec2 value)))
   (direction (:validator (assure-double-vec2 value)))
   (length    (:validator (assure-double-float value))))

  :attributes
  ((span  :uncached (v* (cl:the double-float (the length)) (cl:the vec2 (the direction))))
   (start :uncached (v- (cl:the vec2 (the mid)) (v* 0.5d0 (cl:the vec2 (the span)))))))

(defmethod rm-draw-node ((self 2d-line-mixin))
  (when (the visible?)
    (let ((scene (the krma:scene))
	  (start (the start))
	  (end   (the end))
	  (oid   (the object-id)))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff
	     (list (vx start) (vy start) 0.0d0
		   (vx end)   (vy end)   0.0d0)
	     oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defmethod clui::compute-make-instance-arguments ((protocol 2d-line)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'start provided-inputs)
		(member  'end   provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_a)) initargs))
	  ((and (member 'start provided-inputs)
		(member 'mid   provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_b)) initargs))
	  ((and (member 'mid provided-inputs)
		(member 'end provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_c)) initargs))
	  ((and (member 'start provided-inputs)
		(member 'span  provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_d)) initargs))
	  ((and (member 'start     provided-inputs)
		(member 'direction provided-inputs)
		(member 'length    provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_e)) initargs))
	  ((and (member 'end  provided-inputs)
		(member 'span provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_f)) initargs))
	  ((and (member 'end       provided-inputs)
		(member 'direction provided-inputs)
		(member 'length    provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_g)) initargs))
	  ((and (member 'mid  provided-inputs)
		(member 'span provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_h)) initargs))
	  ((and (member 'mid       provided-inputs)
		(member 'direction provided-inputs)
		(member 'length    provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class '2d-line_i)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args '2d-line ~S" initargs)))))

(defobject 2d-line-offset (2d-line-mixin entity-mixin)
  :inputs
  ((curve (:validator (assert (typep value '2d-line-mixin)) value))
   (distance (:validator (assure-double-float value))))

  :attributes
  ((direction        :uncached (the curve direction))
   (offset-direction :uncached (let ((dir (the direction))) (vec2 (- (vy dir)) (vx dir))))
   (offset-vector    :uncached (v* (cl:the double-float (the distance))
				   (cl:the vec2 (the offset-direction))))
   (start            :uncached (v+ (the curve start) (the offset-vector)))
   (end              :uncached (v+ (the curve end)   (the offset-vector)))
   (mid              :uncached (v+ (the curve mid)   (the offset-vector)))
   (span             :uncached (the curve span))
   (length           :uncached (the curve length))
   (degenerate?      :uncached (the curve degenerate?))
   (domain           :uncached (the curve domain))
   (degree           :uncached (the curve degree))
   (closed?          :uncached (the curve closed?))
   (periodic?        :uncached (the curve periodic?))))
   

(defobject 3d-line-mixin (space-curve-mixin line-mixin)
  :attributes
  ((degenerate? :uncached (v= (cl:the vec3 (the start)) (cl:the vec3 (the end))))
   (start       :uncached (v- (cl:the vec3 (the end)) (cl:the vec3 (the span))))
   (end         :uncached (v+ (cl:the vec3 (the start)) (cl:the vec3 (the span))))
   (span        :uncached (v- (cl:the vec3 (the end)) (cl:the vec3 (the start))))
   (mid         :uncached (v+ (cl:the vec3 (the start)) (v* 0.5d0 (the span))))
   (length      :uncached (vlength (cl:the vec3 (the span))))
   (direction   :uncached (if (the degenerate?)
			      (vec3 0 0 0)
			      (vunit (cl:the vec3 (the span)))))
   (domain      :uncached (list 0.0d0 1.0d0))
   (degree      :uncached 1)
   (closed?     :uncached nil)
   (periodic?   :uncached nil)))

(defmethod centroidish ((self 3d-line-mixin))
  (the mid))

(defmethod to-opencascade ((self 3d-line-mixin))
  (let* ((start (the start))
	 (end   (the end))
	 (tail  (gp:pnt (vx start) (vy start) (vz start)))
	 (head  (gp:pnt (vx end)   (vy end)   (vz end))))
    (oc::value (make-instance 'oc::gc-make-segment
			      :P1 tail
			      :P2 head))))

(defclass line ()
  ()
  (:metaclass clui::protocol-class))

(defobject line_a (3d-line-mixin entity-mixin)
  :inputs
  ((start (:validator (assure-double-vec3 value)))
   (end   (:validator (assure-double-vec3 value)))))

(defobject line_b (3d-line-mixin entity-mixin)
  :inputs
  ((start (:validator (assure-double-vec3 value)))
   (mid   (:validator (assure-double-vec3 value))))

  :attributes
  ((span :uncached (v* 2.0d0 (v- (cl:the vec3 (the mid)) (cl:the vec3 (the start)))))))

(defobject line_c (3d-line-mixin entity-mixin)
  :inputs
  ((mid (:validator (assure-double-vec3 value)))
   (end (:validator (assure-double-vec3 value))))

  :attributes
  ((span :uncached (v* 2.0d0 (v- (cl:the vec3 (the end)) (cl:the vec3 (the mid)))))))

(defobject line_d (3d-line-mixin entity-mixin)
  :inputs
  ((start (:validator (assure-double-vec3 value)))
   (span  (:validator (assure-double-vec3 value)))))

(defobject line_e (3d-line-mixin entity-mixin)
  :inputs
  ((start     (:validator (assure-double-vec3 value)))
   (direction (:validator (assure-double-vec3 value)))
   (length    (:validator (assure-double-float value))))

  :attributes
  ((span :uncached (v* (cl:the double-float (the length)) (cl:the vec3 (the direction))))))

(defobject line_f (3d-line-mixin entity-mixin)
  :inputs
  ((end  (:validator (assure-double-vec3 value)))
   (span (:validator (assure-double-vec3 value)))))

(defobject line_g (3d-line-mixin entity-mixin)
  :inputs
  ((end       (:validator (assure-double-vec3 value)))
   (direction (:validator (assure-double-vec3 value)))
   (length    (:validator (assure-double-float value))))

  :attributes
  ((span :uncached (v* (cl:the double-float (the length)) (cl:the vec3 (the direction))))))

(defobject line_h (3d-line-mixin entity-mixin)
  :inputs
  ((mid  (:validator (assure-double-vec3 value)))
   (span (:validator (assure-double-vec3 value))))

  :attributes
  ((start :uncached (v- (cl:the vec3 (the mid)) (v* 0.5d0 (cl:the vec3 (the span)))))))

(defobject line_i (3d-line-mixin entity-mixin)
  :inputs
  ((mid       (:validator (assure-double-vec3 value)))
   (direction (:validator (assure-double-vec3 value)))
   (length    (:validator (assure-double-float value))))

  :attributes
  ((span  :uncached (v* (cl:the double-float (the length)) (cl:the vec3 (the direction))))
   (start :uncached (v- (cl:the vec3 (the mid)) (v* 0.5d0 (cl:the vec3 (the span)))))))

(defmethod rm-draw-node ((self 3d-line-mixin))
  (when (the visible?)
    (let ((scene (the krma:scene))
	  (start (the start))
	  (end   (the end))
	  (oid   (the object-id)))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff
	     (list (vx start) (vy start) (vz start)
		   (vx end)   (vy end)   (vz end))
	     oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))



(defmethod clui::compute-make-instance-arguments ((protocol line)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'start provided-inputs)
		(member  'end   provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_a)) initargs))
	  ((and (member 'start provided-inputs)
		(member 'mid   provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_b)) initargs))
	  ((and (member 'mid provided-inputs)
		(member 'end provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_c)) initargs))
	  ((and (member 'start provided-inputs)
		(member 'span  provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_d)) initargs))
	  ((and (member 'start     provided-inputs)
		(member 'direction provided-inputs)
		(member 'length    provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_e)) initargs))
	  ((and (member 'end  provided-inputs)
		(member 'span provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_f)) initargs))
	  ((and (member 'end       provided-inputs)
		(member 'direction provided-inputs)
		(member 'length    provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_g)) initargs))
	  ((and (member 'mid  provided-inputs)
		(member 'span provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_h)) initargs))
	  ((and (member 'mid       provided-inputs)
		(member 'direction provided-inputs)
		(member 'length    provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'line_i)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'line ~S" initargs)))))


(defobject conic-section-mixin (plane-curve-mixin))

(defun %circle-from-3pts (start-point interior-point end-point)
  ;; https://web.archive.org/web/20161011113446/http://www.abecedarical.com/zenosamples/zs_circle3pts.html
  (flet ((avg ()
	     (vec2 (/ (+ (vx start-point) (vx interior-point) (vx end-point)) 3.0d0)
		   (/ (+ (vy start-point) (vy interior-point) (vy end-point)) 3.0d0))))
    (if (or (v= start-point interior-point)
	    (v= interior-point end-point)
	    (v= start-point end-point))
	;; points are coincident, degenerate:
	(values (avg) 0.0d0 t)
      
	(labels ((det2x2 (a b
			  c d)
		   (- (* a d) (* b c)))
		 (det3x3 (a b c
			  d e f
			  g h i)
		   (+ (* a (det2x2 e f
				   h i))
		      (- (* b (det2x2 d f
				      g i)))
		      (* c (det2x2 d e
				   g h)))))
	     
	  (let* ((a22 (vx start-point))
		 (a23 (vy start-point))
		 (a21 (+ (* a22 a22) (* a23 a23)))
		 (a24 1.0d0)
		 (a32 (vx interior-point))
		 (a33 (vy interior-point))
		 (a31 (+ (* a32 a32) (* a33 a33)))
		 (a34 1.0d0)
		 (a42 (vx end-point))
		 (a43 (vy end-point))
		 (a41 (+ (* a42 a42) (* a43 a43)))
		 (a44 1.0d0)
		 (m11 (det3x3 a22 a23 a24
			      a32 a33 a34
			      a42 a43 a44)))
	     
	    (if (= 0.0d0 m11)
		;; points are collinear, degenerate:
		(values (avg) most-positive-double-float t)

		;; circle is ok:
		(let* ((m12 (det3x3 a21 a23 a24
				    a31 a33 a34
				    a41 a43 a44))
		       (m13 (det3x3 a21 a22 a24
				    a31 a32 a34
				    a41 a42 a44))
		       (m14 (det3x3 a21 a22 a23
				    a31 a32 a33
				    a41 a42 a43))
		       (center-u (* 0.5 (/ m12 m11)))
		       (center-v (* -0.5 (/ m13 m11)))
		       (radius (sqrt (+ (* center-u center-u) (* center-v center-v) (/ m14 m11)))))
		
		  (values (vec2 center-u center-v) radius nil))))))))

(defobject circle-mixin (conic-section-mixin entity-mixin)
  :attributes
  ((degenerate? :uncached (= 0.0d0 (the radius)))
   (closed?     :uncached t)
   (periodic?   :uncached t)
   (length      :uncached (* 2pi (the radius)))))

(defmethod centroidish ((self circle-mixin))
  (let ((c (the center)))
    (vec3 (vx c) (vy c) 0.0d0)))

(defmethod rm-draw-node ((self circle-mixin))
  (when (the visible?)
    (let* ((scene  (the krma:scene))
	   (center (the center))
	   (radius (the radius))
	   (start-angle 0.0d0)
	   (oid    (the object-id))		      
	   (vertices (loop repeat 200 for i from 0
			   with theta = start-angle
			   with step = (/ 2pi 199.0d0)
			   append (let ((angle (+ theta (* i step))))
				    (list (+ (vx center) (* radius (cos angle)))
					  (+ (vy center) (* radius (sin angle)))
					  0.0d0)))))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff vertices oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defclass circle ()
  ()
  (:metaclass clui::protocol-class))

(defobject circle_a (circle-mixin)
  :inputs
  ((center (:validator (assure-double-vec2 value)))
   (radius (:validator (assure-nonnegative-double-float value)))))

(defobject circle_b (circle-mixin)
  :inputs
  ((first   (:validator (assure-double-vec2 value)))
   (second  (:validator (assure-double-vec2 value)))
   (third   (:validator (assure-double-vec2 value))))

  :attributes
  ((%3pt-circle-info (multiple-value-list
		      (%circle-from-3pts (the first) (the second) (the third))))
   (center :uncached (first (the %3pt-circle-info)))
   (radius :uncached (second (the %3pt-circle-info)))
   (degenerate? :uncached (third (the %3pt-circle-info)))))

(defobject circle_c (circle-mixin)
  :inputs
  ((start    (:validator (assure-double-vec2 value)))
   (interior (:validator (assure-double-vec2 value)))
   (end      (:validator (assure-double-vec2 value))))

  :attributes
  ((%3pt-circle-info (multiple-value-list
		      (%circle-from-3pts (the start) (the interior) (the end))))
   (center :uncached (first (the %3pt-circle-info)))
   (radius :uncached (second (the %3pt-circle-info)))
   (degenerate? :uncached (third (the %3pt-circle-info)))))

(defmethod clui::compute-make-instance-arguments ((protocol circle)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'center provided-inputs)
		(member 'radius provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'circle_a)) initargs))
	  ((and (member 'first  provided-inputs)
		(member 'second provided-inputs)
		(member 'third  provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'circle_b)) initargs))
	  ((and (member 'start    provided-inputs)
		(member 'interior provided-inputs)
		(member 'end      provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'circle_c)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'circle ~S" initargs)))))
   
(defobject ellipse-mixin (conic-section-mixin entity-mixin)
  :attributes
  ((degenerate?     :uncached (or (= 0.0d0 (the semi-major-axis))
				  (= 0.0d0 (the semi-minor-axis))))
   
   (closed?         :uncached t)
   
   (periodic?       :uncached t)
   
   (circular?       :uncached (v= (the focus1) (the focus2)))
   
   (center          :uncached (v* 0.5d0 (v+ (the focus1) (the focus2))))
   
   (focal-distance  :uncached (vlength (v- (the focus1) (the center))))
   
   (major-axis      :uncached (* 2.0d0 (the semi-major-axis)))

   (semi-major-axis :uncached (* 0.5d0 (the major-axis)))
   
   (semi-minor-axis :uncached (sqrt (- (expt (the semi-major-axis) 2)
				       (expt (the focal-distance) 2))))
   
   (eccentricity    :uncached (/ (sqrt (- (expt (the semi-major-axis) 2)
					  (expt (the semi-minor-axis) 2)))
				 (the semi-major-axis)))
   
   (h               :uncached (/ (expt (- (the semi-major-axis) (the semi-minor-axis)) 2)
				 (expt (+ (the semi-major-axis) (the semi-minor-axis)) 2)))
   
   (length          :uncached (* pi (+ (the semi-major-axis) (the semi-minor-axis))
				 (let ((h (the h)))
				   (loop for n from 0 below 10
					 sum (* (binomial-coefficient-0.5 n) (expt h n))))))))

(defmethod centroidish ((self ellipse-mixin))
  (let ((c (the center)))
    (vec3 (vx c) (vy c) 0.0d0)))

(defclass ellipse ()
  ()
  (:metaclass clui::protocol-class))

(defobject ellipse_a (ellipse-mixin)
  :inputs
  ((focus1          (:validator (assure-double-vec2 value)))
   (focus2          (:validator (assure-double-vec2 value)))
   (semi-major-axis (:validator (assure-nonnegative-double-float value)))))

(defobject ellipse_b (ellipse-mixin)
  :inputs
  ((focus1     (:validator (assure-double-vec2 value)))
   (focus2     (:validator (assure-double-vec2 value)))
   (major-axis (:validator (assure-nonnegative-double-float value)))))

(defobject ellipse_c (ellipse-mixin)
  :inputs
  ((center          (:validator (assure-double-vec2 value)))
   (focus1          (:validator (assure-double-vec2 value)))
   (semi-major-axis (:validator (assure-nonnegative-double-float value))))
  
  :attributes
  ((focus2 :uncached (v+ (the center) (v- (the center) (the focus1))))))

(defobject ellipse_d (ellipse-mixin)
  :inputs
  ((center (:validator (assure-double-vec2 value)))
   (focus1 (:validator (assure-double-vec2 value)))
   (major-axis (:validator (assure-nonnegative-double-float value))))
  
  :attributes
  ((focus2 :uncached (v+ (the center) (v- (the center) (the focus1))))))

(defobject ellipse_e (ellipse-mixin)
  :inputs
  ((center (:validator (assure-double-vec2 value)))
   (focus2 (:validator (assure-double-vec2 value)))
   (semi-major-axis (:validator (assure-nonnegative-double-float value))))

  :attributes
  ((focus1 :uncached (v+ (the center) (v- (the center) (the focus2))))))

(defobject ellipse_f (ellipse-mixin)
  :inputs
  ((center (:validator (assure-double-vec2 value)))
   (focus2 (:validator (assure-double-vec2 value)))
   (major-axis (:validator (assure-nonnegative-double-float value))))

  :attributes
  ((focus1 :uncached (v+ (the center) (v- (the center) (the focus2))))))

(defmethod clui::compute-make-instance-arguments ((protocol ellipse)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'focus1 provided-inputs)
		(member 'focus2 provided-inputs)
		(member 'semi-major-axis provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'ellipse_a)) initargs))
	  ((and (member 'focus1 provided-inputs)
		(member 'focus2 provided-inputs)
		(member 'major-axis provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'ellipse_b)) initargs))
	  ((and (member 'center provided-inputs)
		(member 'focus1 provided-inputs)
		(member 'semi-major-axis provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'ellipse_c)) initargs))
	  ((and (member 'center provided-inputs)
		(member 'focus1 provided-inputs)
		(member 'major-axis provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'ellipse_d)) initargs))
	  ((and (member 'center provided-inputs)
		(member 'focus2 provided-inputs)
		(member 'semi-major-axis provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'ellipse_e)) initargs))
	  ((and (member 'center provided-inputs)
		(member 'focus2 provided-inputs)
		(member 'major-axis provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'ellipse_f)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'ellipse ~S" initargs)))))

(defobject arc-mixin (conic-section-mixin entity-mixin)
  :attributes
  ((degenerate? :uncached (or (= 0.0d0 (the radius))
			      (= (the end-angle) (the start-angle))))
   (closed?     :uncached (= 2pi (- (the end-angle) (the start-angle))))
   (periodic?   :uncached t)
   (mid         :uncached (let ((mid-angle (+ (the start-angle) (/ (the delta-angle) 2.0d0)))
				(radius (the radius)))
			    (v+ (the center) (vec2 (* radius (cos mid-angle))
						   (* radius (sin mid-angle))))))
   (%center-to-start :uncached (v- (the start) (the center)))
   (%center-to-end   :uncached (v- (the end)   (the center)))
   (radius :uncached (min (vlength (the %center-to-start))
			  (vlength (the %center-to-end))))
   (start-angle :uncached (let ((vec (the %center-to-start)))
			    (atan (vy vec) (vx vec))))
   (start       :uncached (let ((sa (the start-angle)))
			    (v+ (the center) (v* (the radius) (vec2 (cos sa) (sin sa))))))
   (end-angle   :uncached (let ((vec (the %center-to-end)))
			    (atan (vy vec) (vx vec))))
   (end         :uncached (let ((ea (the end-angle)))
			    (v+ (the center) (v* (the radius) (vec2 (cos ea) (sin ea))))))   
   (delta-angle :uncached (let* ((start (the start-angle))
				 (end (the end-angle))
				 (a (- end start)))
			    (if (eq (the orientation) :ccw)
				;; ccw is positive delta
				(if (< a 0)
				    (+ 2pi a)
				    a)
				;; cw is negative delta
				(if (< a 0)
				    a
				    (- a 2pi)))))
   (length :uncached (* (abs (the delta-angle)) (the radius)))
   (domain :uncached (let ((start (the start-angle)))
		       (if (eq (the orientation) :ccw)
			   (list (mod start 2pi) (+ (mod start 2pi) (the delta-angle)))
			   (list (+ (mod start 2pi) (the delta-angle)) (mod start 2pi)))))))

(defmethod centroidish ((self arc-mixin))
  (let* ((c (the center))
	 (val (v* (the radius) (vec3 (- (sin (the end-angle)) (sin (the start-angle)))
				     (- (cos (the start-angle)) (cos (the end-angle)))
				     0.0d0))))
    (if (eq (the orientation) :ccw)
	(v+ (vec3 (vx c) (vy c) 0.0d0) val)
	(v- (vec3 (vx c) (vy c) 0.0d0) val))))

(defmethod to-opencascade ((self arc-mixin))
  (let* ((start (the start))
	 (mid   (the mid))
	 (end   (the end))
	 (p1 (gp:pnt (vx start) (vy start) 0.0d0))
	 (p2 (gp:pnt (vx mid)   (vy mid)   0.0d0))
	 (p3 (gp:pnt (vx end)   (vy end)   0.0d0)))
    (oc::value (make-instance 'oc::gc-make-arc-of-circle
			      :P1 p1
			      :P2 p2
			      :P3 p3))))    

(defmethod rm-draw-node ((self arc-mixin))
  (when (the visible?)
    (let* ((scene  (the krma:scene))
	   (center (the center))
	   (radius (the radius))
	   (start-angle (the start-angle))
	   (oid    (the object-id))		      
	   (vertices (loop repeat 200 for i from 0
			   with theta = start-angle
			   with step = (/ (the delta-angle) 199.0d0)
			   append (let ((angle (+ theta (* i step))))
				    (list (+ (vx center) (* radius (cos angle)))
					  (+ (vy center) (* radius (sin angle)))
					  0.0d0)))))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff vertices oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defmethod evaluate1 ((self arc-mixin) (parameter real) &key (derivs 0))
  (let ((c (cos parameter))
	(s (sin parameter)))
    (case derivs
      (0 (v+ (the center) (v* (the radius) (vec2 c s))))
      (otherwise (values (v+ (the center) (v* (the radius) (vec2 c s))) (vec2 (- s) c))))))

(defclass arc ()
  ()
  (:metaclass clui::protocol-class))

(defobject arc_a (arc-mixin)
  :inputs
  ((center (:validator (assure-double-vec2 value)))
   (start  (:validator (assure-double-vec2 value)))
   (end    (:validator (assure-double-vec2 value)))
   (orientation (:validator (assert (or (eq value :ccw) (eq value :cw))) value) :ccw)))

(defobject arc_b (arc-mixin)
  :inputs
  ((center      (:validator (assure-double-vec2 value)))
   (radius      (:validator (assure-nonnegative-double-float value)))
   (start-angle (:validator (assure-double-float value)))
   (end-angle   (:validator (assure-double-float value)))
   (orientation (:validator (assert (or (eq value :ccw) (eq value :cw))) value) :ccw)))

(defobject arc_c (arc-mixin)
  :inputs
  ((start    (:validator (assure-double-vec2 value)))
   (interior (:validator (assure-double-vec2 value)))
   (end      (:validator (assure-double-vec2 value))))

  :attributes
  ((%3pt-circle-info (multiple-value-list
		      (%circle-from-3pts (the start) (the interior) (the end))))
   (center :uncached (first (the %3pt-circle-info)))
   (radius :uncached (second (the %3pt-circle-info)))
   (degenerate? :uncached (third (the %3pt-circle-info)))
   (%start-to-interior :uncached
		       (let ((interior (the interior))
			     (start (the start)))
			 (vec3 (- (vx interior) (vx start))
			       (- (vy interior) (vy start))
			       0)))
   (%interior-to-end :uncached
		     (let ((end (the end))
			   (interior (the interior)))
		       (vec3 (- (vx end) (vx interior))
			     (- (vy end) (vy interior))
			     0)))
   (orientation (if (plusp (vz (vc (the %start-to-interior) (the %interior-to-end))))
		    :ccw
		    :cw))))

(defobject arc_d (arc-mixin)
  :inputs
  ((first  (:validator (assure-double-vec2 value)))
   (second (:validator (assure-double-vec2 value)))
   (third  (:validator (assure-double-vec2 value))))

  :attributes
  ((start :uncached (the first))
   (interior :uncached (the second))
   (end :uncached (the third))
   (%3pt-circle-info (multiple-value-list
		      (%circle-from-3pts (the first) (the second) (the third))))
   (center :uncached (first (the %3pt-circle-info)))
   (radius :uncached (second (the %3pt-circle-info)))
   (degenerate? :uncached (third (the %3pt-circle-info)))
   (%start-to-interior :uncached
		       (let ((interior (the interior))
			     (start (the start)))
			 (vec3 (- (vx interior) (vx start))
			       (- (vy interior) (vy start))
			       0)))
   (%interior-to-end :uncached
		     (let ((end (the end))
			   (interior (the interior)))
		       (vec3 (- (vx end) (vx interior))
			     (- (vy end) (vy interior))
			     0)))
   (orientation (if (plusp (vz (vc (the %start-to-interior) (the %interior-to-end))))
		    :ccw
		    :cw))))

(defmethod clui::compute-make-instance-arguments ((protocol arc)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'center provided-inputs)
		(member 'start provided-inputs)
		(member 'end provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'arc_a)) initargs))
	  ((and (member 'center provided-inputs)
		(member 'radius provided-inputs)
		(member 'start-angle provided-inputs)
		(member 'end-angle provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'arc_b)) initargs))
	  ((and (member 'start    provided-inputs)
		(member 'interior provided-inputs)
		(member 'end      provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'arc_c)) initargs))
	  ((and (member 'first  provided-inputs)
		(member 'second provided-inputs)
		(member 'third  provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'arc_d)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'arc ~S" initargs)))))

(defobject parametric-curve-mixin (curve-mixin))

(defobject parametric-plane-curve-mixin (plane-curve-mixin parametric-curve-mixin))

(defmethod rm-draw-node ((self parametric-plane-curve-mixin))
  (when (the visible?)
    (let* ((scene  (the krma:scene))
	   (oid    (the object-id))
	   (domain (the domain))
	   (u0     (first domain))
	   (un     (second domain))
	   (vertices (nconc
		      (loop for i from 0 below 100
			    with u = u0
			    with step = (/ (- un u0) 100.0d0)
			    nconc (let ((vec2 (evaluate1 self u)))
				    (incf u step)
				    (list (vx vec2) (vy vec2) 0.0d0)))
		      (let ((vec2 (evaluate1 self un)))
			(list (vx vec2) (vy vec2) 0.0d0)))))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff vertices oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defobject parametric-space-curve-mixin (space-curve-mixin parametric-curve-mixin))

(defmethod rm-draw-node ((self parametric-space-curve-mixin))
  (when (the visible?)
    (let* ((scene  (the krma:scene))
	   (oid    (the object-id))
	   (domain (the domain))
	   (u0     (first domain))
	   (un     (second domain))
	   (vertices (nconc
		      (loop for i from 0 below 100
			    with u = u0
			    with step = (/ (- un u0) 100.0d0)
			    nconc (let ((vec3 (evaluate1 self u)))
				    (incf u step)
				    (list (vx vec3) (vy vec3) (vz vec3))))
		      (let ((vec3 (evaluate1 self un)))
			(list (vx vec3) (vy vec3) (vz vec3))))))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff vertices oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defobject rational-power-basis-plane-curve-mixin (parametric-plane-curve-mixin))

(defobject rational-power-basis-plane-curve (rational-power-basis-plane-curve-mixin entity-mixin)
  :inputs
  (wx-polynomial
   wy-polynomial
   w-polynomial
   domain))

(defun evaluate-polynomial (p x)
  (loop for i from 0 below (length p)
	sum (* (aref p i) (expt x i))))

(defmethod evaluate1 ((self rational-power-basis-plane-curve) (parameter real) &key (derivs 0))
  (declare (ignorable derivs))
  (let ((domain (the domain)))
    (declare (ignorable domain))
    ;;(assert (<= (first domain) parameter (second domain)))
    (let* ((wxpoly (the wx-polynomial))
	   (wypoly (the wy-polynomial))
	   (wpoly (the w-polynomial))
	   (w (evaluate-polynomial wpoly parameter)))
      (vec2 (/ (evaluate-polynomial wxpoly parameter) w)
	    (/ (evaluate-polynomial wypoly parameter) w)))))

(defobject bezier-mixin ())

(defobject bezier-curve-mixin (bezier-mixin parametric-curve-mixin)
  :attributes
  ((order :uncached (length (the homogeneous-control-points)))
   (degree :uncached (1- (the order)))
   (domain :uncached (list 0.0d0 1.0d0))
   (degenerate? :uncached (let ((cpoints (the homogeneous-control-points)))
			    (loop for i from 0 below (the degree)
				  always (v= (elt cpoints i) (elt cpoints (1+ i))))))
   (closed? :uncached (v= (elt (the homogeneous-control-points) 0)
			  (elt (the homogeneous-control-points) (the degree))))
   (periodic? :uncached nil)))

(defobject bezier-space-curve-mixin (bezier-curve-mixin parametric-space-curve-mixin)
  :attributes
  ((rational? :uncached (let ((cpoints (the homogeneous-control-points)))
			  (not
			   (loop for cpoint across cpoints
				 always (= (vw cpoint) 1.0d0)))))))

(defmethod centroidish ((self bezier-space-curve-mixin))
  (let ((cpoints (the control-points)))
    (loop for cp across cpoints
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum (vec3 (vx cp) (vy cp) (vz cp))))
	  finally (return (v/ sum (length cpoints))))))

(declaim (ftype (function (fixnum double-float) (#-allegro simple-array #+allegro array double-float)) bernstein))
(defun bernstein (n u)
  (declare (type fixnum n) (type double-float u))
  (declare (optimize (speed 3) (safety 1)))
  (let ((B (make-array (1+ n) :element-type 'double-float :initial-element 1.0d0))
	(u1 (- 1.0d0 u))
	(saved 0.0d0)
	(temp 0.0d0))
    (declare (type double-float saved temp))
    (loop for j from 1 to n
	  do (setq saved 0.0d0)
	     (loop for k from 0 below j
		   do (setq temp (aref B k))
		      (setf (aref B k) (+ saved (* u1 temp)))
		      (setq saved (* u temp)))
	     (setf (aref B j) saved)
	  finally (return B))))

(defmethod evaluate1 ((self bezier-space-curve-mixin) (parameter real) &key (derivs 0))
  (declare (ignorable derivs))
  ;; todo: make work for rational curves
  (declare (optimize (speed 3) (safety 3)))
  (setq parameter (coerce parameter 'double-float))
  (let* ((n (the degree)))
    (declare (type fixnum n))
    (let* ((B (bernstein n parameter))
	   (Cw (3dm.d::%vec4 0.0d0 0.0d0 0.0d0 0.0d0))
	   (P (the homogeneous-control-points)))
      (loop for k from 0 to n
	    do (setq Cw (v+ (cl:the vec4 Cw) (v* (cl:the double-float (aref B k)) (cl:the vec4 (elt P k)))))
	    finally (return (safe-euclid Cw))))))

(defobject bezier-plane-curve-mixin (bezier-curve-mixin parametric-plane-curve-mixin)
  :attributes
  ((rational? :uncached (let ((cpoints (the homogeneous-control-points)))
			  (not
			   (loop for cpoint across cpoints
				 always (= (vz cpoint) 1.0d0)))))))

(defmethod centroidish ((self bezier-plane-curve-mixin))
  (let ((cpoints (the control-points)))
    (loop for cp across cpoints
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum (vec3 (vx cp) (vy cp) 0.0d0)))
	  finally (return (v/ sum (length cpoints))))))

(defmethod evaluate1 ((self bezier-plane-curve-mixin) (parameter real) &key (derivs 0))
  (declare (ignorable derivs))
  ;; todo: make work for rational curves
  (declare (optimize (speed 3) (safety 3)))
  (setq parameter (coerce parameter 'double-float))
  (let* ((n (the degree)))
    (declare (type fixnum n))
    (let* ((B (bernstein n parameter))
	   (C (3dm.d::%vec3 0.0d0 0.0d0 0.0d0))
	   (P (the homogeneous-control-points)))
      (loop for k from 0 to n
	    do (setq C (v+ (cl:the vec3 C) (v* (cl:the double-float (aref B k)) (cl:the vec3 (elt P k)))))
	    finally (return (3dm.d::%vec2 (/ (vx C) (vz C)) (/ (vy C) (vz C))))))))

(defclass bezier-plane-curve ()
  ()
  (:metaclass clui::protocol-class))

(defobject bezier-plane-curve_a (bezier-plane-curve-mixin entity-mixin)
  :inputs
  ((homogeneous-control-points
    (:validator (map 'vector #'(lambda (cp)
				 (assure-double-vec3 cp)) value))))
  
  :attributes
  ((control-points :uncached (map 'vector #'(lambda (p)
					      (vec2 (/ (vx p) (vz p)) (/ (vy p) (vz p))))
				  (the homogeneous-control-points)))
   (weights :uncached (map 'vector #'(lambda (p) (vz p)) (the homogeneous-control-points)))))

(defobject bezier-plane-curve_b (bezier-plane-curve-mixin entity-mixin)
  :inputs
  ((control-points
    (:validator (map 'vector #'(lambda (cp)
				 (assure-double-vec2 cp)) value)))
   (weights
    (:validator (map 'vector #'(lambda (w)
				 (assure-double-float w)) value))
    (make-array (length (the control-points)) :initial-element 1.0d0)))

  :attributes
  ((homogeneous-control-points (map 'vector #'(lambda (cp w)
						(vec3 (* w (vx cp)) (* w (vy cp)) w))
				    (the control-points)
				    (the weights)))))

(defmethod clui::compute-make-instance-arguments ((protocol bezier-plane-curve)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'control-points provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'bezier-plane-curve_b)) initargs))
	  ((and (member 'homogeneous-control-points provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'bezier-plane-curve_a)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'bezier-plane-curve ~S" initargs)))))

(defclass bezier-space-curve ()
  ()
  (:metaclass clui::protocol-class))

(defobject bezier-space-curve_a (bezier-space-curve-mixin entity-mixin)
  :inputs
  ((homogeneous-control-points
    (:validator (map 'vector #'(lambda (cp)
				 (assure-double-vec4 cp)) value))))
  
  :attributes
  ((control-points :uncached (map 'vector #'(lambda (p)
					      (vec3 (/ (vx p) (vw p)) (/ (vy p) (vw p)) (/ (vz p) (vw p))))
				  (the homogeneous-control-points)))
   (weights :uncached (map 'vector #'(lambda (p) (vw p)) (the homogeneous-control-points)))))

(defobject bezier-space-curve_b (bezier-space-curve-mixin entity-mixin)
  :inputs
  ((control-points
    (:validator (map 'vector #'(lambda (cp)
				 (assure-double-vec3 cp)) value)))
   (weights
    (:validator (map 'vector #'(lambda (w)
				 (assure-double-float w)) value))
    (make-array (length (the control-points)) :initial-element 1.0d0)))

  :attributes
  ((homogeneous-control-points (map 'vector #'(lambda (cp w)
						(vec4 (* w (vx cp)) (* w (vy cp)) (* w (vz cp)) w))
				    (the control-points)
				    (the weights)))))

(defmethod clui::compute-make-instance-arguments ((protocol bezier-space-curve)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'control-points provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'bezier-space-curve_b)) initargs))
	  ((and (member 'homogeneous-control-points provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'bezier-space-curve_a)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'bezier-space-curve ~S" initargs)))))

(defobject b-spline-mixin ())

(defobject b-spline-curve-mixin (b-spline-mixin parametric-curve-mixin)
  :attributes
  ((degree :uncached (let* ((m (1- (length (the knot-vector))))
			    (cpoints (the homogeneous-control-points))
			    (n (1- (length cpoints))))
		       ;; m = n + p + 1
		       (- m n 1)))
   (order :uncached (1+ (the degree)))
   (bezier? :uncached (and (= (the degree) (1- (length (the homogeneous-control-points))))
			   (let* ((knot-vector (the knot-vector))
				  (m+1 (length knot-vector)))
			     (and (loop for i from 0 below (/ m+1 2)
					always (= 0.0d0 (aref knot-vector i)))
				  (loop for i from (/ m+1 2) below m+1
					always (= 1.0d0 (aref knot-vector i)))))))
   (domain :uncached (let ((knot-vector (the knot-vector)))
		       (list (aref knot-vector 0) (aref knot-vector (1- (length knot-vector))))))
   (degenerate? :uncached (let ((cpoints (the homogeneous-control-points)))
			    (loop for i from 0 to (- (length cpoints) 2)
				  always (v= (elt cpoints i) (elt cpoints (1+ i))))))
   (closed? :uncached (v= (the start) (the end)))
   (uniform? :uncached (let* ((kv (the knot-vector))
			      (ds (- (aref kv 1) (aref kv 0))))
			 (loop for i from 1 to (- (length (the homogeneous-control-points)) 2)
			       always (= ds (- (aref kv (1+ i)) (aref kv i))))))
   (clamped? :uncached (let ((kv (the knot-vector))
			    (order (the order)))
			(and (apply #'= (loop for i from 0 below order
					      collect (elt kv i)))
		             (apply #'= (let ((m (1- (length kv))))
			                  (loop for i from 0 below order
						collect (elt kv (- m i))))))))
   (periodic? :uncached (and (not (the clamped?)) (the uniform?)))
   (start :uncached (evaluate1 self (first (the domain))))
   (end :uncached (evaluate1 self (second (the domain))))))

(defobject b-spline-space-curve-mixin (b-spline-curve-mixin parametric-space-curve-mixin)
  :attributes
  ((rational? :uncached (let ((cpoints (the homogeneous-control-points)))
			  (not
			   (loop for cpoint across cpoints
				 always (= (vw cpoint) 1.0d0)))))

   (control-points :uncached (map 'vector #'(lambda (p)
					      (vec3 (/ (vx p) (vw p)) (/ (vy p) (vw p)) (/ (vz p) (vw p))))
				  (the homogeneous-control-points)))
   
   (weights :uncached (map 'vector #'(lambda (p) (vw p)) (the homogeneous-control-points)))
   
   (homogeneous-control-points (map 'vector #'(lambda (cp w)
						(vec4 (* w (vx cp)) (* w (vy cp)) (* w (vz cp)) w))
				    (the control-points)
				    (the weights)))))

(defmethod centroidish ((self b-spline-space-curve-mixin))
  (let ((cpoints (the control-points)))
    (loop for cp across cpoints
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum (vec3 (vx cp) (vy cp) (vz cp))))
	  finally (return (v/ sum (length cpoints))))))

(defmethod to-opencascade ((self b-spline-space-curve-mixin))
  (let* ((hcpoints (the homogeneous-control-points))
	 (number-of-poles (length hcpoints))
	 (poles (let ((array (make-instance 'oc::TCol-gp-Array1-Of-Pnt
					    :theLower 1 :theUpper number-of-poles)))
		  (loop for i from 1 to number-of-poles
			for j from 0
			do (let ((gp-pnt (gp:pnt (vx (aref hcpoints j))
						 (vy (aref hcpoints j))
						 (vz (aref hcpoints j)))))
			     (oc::set-value array i gp-pnt))
			finally (return array))))
	 (weights (let ((array (make-instance 'oc::TCol-Std-Array1-Of-Real
					      :theLower 1 :theUpper number-of-poles)))
		    (loop for i from 1 to number-of-poles
			  for j from 0
			  do (oc::set-value array i (vw (aref hcpoints j)))
			  finally (return array)))))
    
    (multiple-value-bind (%knots %multiplicities)
	(loop for knot across (the knot-vector)
	      for i from 0
	      with multiplicity = 1
	      with last-knot = nil
	      with multiplicities = ()
	      with knots = ()
	      do (when (zerop i) (push knot knots))
	      when last-knot
		do (if (= last-knot knot)
		       (incf multiplicity)
		       (progn
			 (push multiplicity multiplicities)
			 (push knot knots)
			 (setq multiplicity 1)))
	      do (setq last-knot knot)
	      finally (push multiplicity multiplicities)
		      (return (values (nreverse knots) (nreverse multiplicities))))
      (let ((knots (let ((array (make-instance 'oc::TCol-Std-Array1-Of-Real
					       :theLower 1 :theUpper (length %knots))))
		     (loop for i from 1 for knot in %knots
			   do (oc::set-value array i knot)
			   finally (return array))))
	    (multiplicities (let ((array (make-instance 'oc::TCol-Std-Array1-Of-Integer
							:theLower 1 :theUpper (length %multiplicities))))
			      (loop for i from 1 for m in %multiplicities
				    do (oc::set-value array i m)
				    finally (return array)))))
	(make-instance 'oc::geom-bspline-curve
		       :Poles poles
		       :Weights weights
		       :Knots knots
		       :Multiplicities multiplicities
		       :Degree (the degree)
		       :Periodic (the periodic?))))))

(defun find-span (n p u knot-vector)
  ;; Algorithm A2.1 pg. 68 _The Nurbs Book_ 2nd edition
  (when (= u (elt knot-vector (1+ n)))
    (return-from find-span n))
  (let* ((low p)
	 (high (1+ n))
	 (mid (floor (+ low high) 2)))
    (loop while (or (< u (aref knot-vector mid))
		    (>= u (aref knot-vector (1+ mid))))
	  do (if (< u (aref knot-vector mid))
		 (setq high mid)
		 (setq low mid))
	     (setq mid (floor (+ low high) 2))
	  finally (return mid))))

(defun basis-funs (i u p knot-vector)
  ;; Algorithm A2.2 pg. 70 _The Nurbs Book_ 2nd edition
  (let ((N (make-array (1+ p) :initial-element 1.0d0))
	(left (make-array (1+ p)))
	(right (make-array (1+ p)))
	(saved 0.0d0)
	(temp 0.0d0))
    
    (loop for j from 1 to p
	  do (setf (aref left j) (- u (aref knot-vector (- (1+ i) j))))
	     (setf (aref right j) (- (aref knot-vector (+ i j)) u))
	     (setq saved 0.0d0)
	     
	     (loop for r from 0 below j
		   do (setq temp (/ (aref N r) (+ (aref right (1+ r)) (aref left (- j r)))))
		      (setf (aref N r) (+ saved (* (aref right (1+ r)) temp)))
		      (setq saved (* (aref left (- j r)) temp)))

	     (setf (aref N j) saved)
	  finally (return N))))

(defun ders-basis-funs (i u p n knot-vector)
  ;; Algorithm A2.3, The Nurbs Book, 2nd edition, p. 72
  "Compute nonzero basis functions and their derivatives."
  (setq u (coerce u 'double-float))
  (let* ((1+p (1+ p))
	 (ders (make-array (list 1+p 1+p) :initial-element 0.0d0))
	 (ndu  (make-array (list 1+p 1+p) :initial-element 1.0d0))
	 (left  (make-array 1+p))
	 (right (make-array 1+p))
	 (saved 0.0d0)
	 (temp 0.0d0))
    (loop for j from 1 to p
	  do (setf (aref left j) (- u (aref knot-vector (1+ (- i j))))
		   (aref right j) (- (aref knot-vector (+ i j)) u))
	     (setq saved 0.0d0)
	     (loop for r from 0 below j
		   do ;; lower triangle:
		      (setf (aref ndu j r) (+ (aref right (1+ r)) (aref left (- j r))))
		      (setq temp (/ (aref ndu r (1- j)) (aref ndu j r)))
		      ;; upper triangle:
		      (setf (aref ndu r j) (+ saved (* (aref right (1+ r)) temp)))
		      (setq saved (* (aref left (- j r)) temp)) )
	     (setf (aref ndu j j) saved))
    ;; load the basis functions:
    (loop for j from 0 to p
	  do (setf (aref ders 0 j) (aref ndu j p)))
    ;; This section computes the derivatives
    (let ((a (make-array (list 2 (1+ n)) :initial-element 1.0d0)))
      (loop for r from 0 to p ;; loop over function index
	    do (let ((s1 0) (s2 1))
		 ;;(setf (aref a 0 0) 1.0d0)
		 ;; loop to compute kth derivative
		 (loop for k from 1 to n
		       do (let ((d 0.0d0)
				(rk (- r k))
				(pk (- p k))
				(j1)
				(j2)
				(j))
			    (when (>= r k)
			      (setf (aref a s2 0) (/ (aref a s1 0) (aref ndu (+ pk 1) rk)))
			      (setq d (* (aref a s2 0) (aref ndu rk pk))))
			    (if (>= rk -1)
				(setq j1 1)
				(setq j1 (- rk)))
			    (if (<= (1- r) pk)
				(setq j2 (1- k))
				(setq j2 (- p r)))
			    (loop for j from j1 to j2
				  do (setf (aref a s2 j) (/ (- (aref a s1 j) (aref a s1 (1- j))) (aref ndu (1+ pk) (+ rk j))))
				     (incf d (* (aref a s2 j) (aref ndu (+ rk j) pk))))
			    (when (<= r pk)
			      (setf (aref a s2 k) (/ (- (aref a s1 (1- k))) (aref ndu (1+ pk) r)))
			      (incf d (* (aref a s2 k) (aref ndu r pk))))
			    (setf (aref ders k r) d)
			    (setq j s1
				  s1 s2
				  s2 j) ;; switch rows
			    )))))
    ;; multiply through by the correct factors
    (let ((r p))
      (loop for k from 1 to n
	    do (loop for j from 0 to p
		     do (setf (aref ders k j) (* (aref ders k j) r)))
	       (setq r (* r (- p k)))))
    ders))

(defmethod evaluate1 ((self b-spline-space-curve-mixin) (parameter real) &key (derivs 0))
  (declare (ignorable derivs))
  (let* ((knot-vector (the knot-vector))
	 (cpoints (the homogeneous-control-points))
	 (n (1- (length cpoints)))
	 (p (the degree))
	 (span (find-span n p parameter knot-vector))
	 (basis-funs (basis-funs span parameter p knot-vector)))
    (let ((Cw (vec4 0 0 0 0)))
      (loop for i from 0 to p
	    do (setq Cw (v+ Cw (v* (aref basis-funs i)
				   (aref cpoints (+ (- span p) i)))))
	    finally (return (safe-euclid Cw))))))

(defobject b-spline-plane-curve-mixin (b-spline-curve-mixin parametric-plane-curve-mixin)
  :attributes
  ((rational? :uncached (let ((cpoints (the homogeneous-control-points)))
			  (not
			   (loop for cpoint across cpoints
				 always (= (vz cpoint) 1.0d0)))))
   
   (control-points :uncached (map 'vector #'(lambda (p)
					      (vec2 (/ (vx p) (vz p)) (/ (vy p) (vz p))))
				  (the homogeneous-control-points)))
   
   (weights :uncached (map 'vector #'(lambda (p) (vz p)) (the homogeneous-control-points)))

   (homogeneous-control-points (map 'vector #'(lambda (cp w)
						(vec3 (* w (vx cp)) (* w (vy cp)) w))
				    (the control-points)
				    (the weights)))))

(defmethod centroidish ((self b-spline-plane-curve-mixin))
  (let ((cpoints (the control-points)))
    (loop for cp across cpoints
	  with sum = (vec3 0 0 0)
	  do (setq sum (v+ sum (vec3 (vx cp) (vy cp) 0.0d0)))
	  finally (return (v/ sum (length cpoints))))))

(defmethod to-opencascade ((self b-spline-plane-curve-mixin))
  (let* ((hcpoints (the homogeneous-control-points))
	 (number-of-poles (length hcpoints))
	 (poles (let ((array (make-instance 'oc::TCol-gp-Array1-Of-Pnt
					    :theLower 1 :theUpper number-of-poles)))
		  (loop for i from 1 to number-of-poles
			for j from 0
			do (let* ((gp-pnt (gp:pnt (vx (aref hcpoints j))
						  (vy (aref hcpoints j))
						  0.0d0)))
			     (oc::set-value array i gp-pnt))
			finally (return array))))
	 (weights (let ((array (make-instance 'oc::TCol-Std-Array1-Of-Real
					      :theLower 1 :theUpper number-of-poles)))
		    (loop for i from 1 to number-of-poles
			  for j from 0
			  do (oc::set-value array i (vz (aref hcpoints j)))
			  finally (return array)))))
    
    (multiple-value-bind (%knots %multiplicities)
	(loop for knot across (the knot-vector)
	      for i from 0
	      with multiplicity = 1
	      with last-knot = nil
	      with multiplicities = ()
	      with knots = ()
	      do (when (zerop i) (push knot knots))
	      when last-knot
		do (if (= last-knot knot)
		       (incf multiplicity)
		       (progn
			 (push multiplicity multiplicities)
			 (push knot knots)
			 (setq multiplicity 1)))
	      do (setq last-knot knot)
	      finally (push multiplicity multiplicities)
		      (return (values (nreverse knots) (nreverse multiplicities))))
      (let ((knots (let ((array (make-instance 'oc::TCol-Std-Array1-Of-Real
					       :theLower 1 :theUpper (length %knots))))
		     (loop for i from 1 for knot in %knots
			   do (oc::set-value array i knot)
			   finally (return array))))
	    (multiplicities (let ((array (make-instance 'oc::TCol-Std-Array1-Of-Integer
							:theLower 1 :theUpper (length %multiplicities))))
			      (loop for i from 1 for m in %multiplicities
				    do (oc::set-value array i m)
				    finally (return array)))))
	(make-instance 'oc::geom-bspline-curve
		       :Poles poles
		       :Weights weights
		       :Knots knots
		       :Multiplicities multiplicities
		       :Degree (the degree)
		       :Periodic (the periodic?))))))


    

(defmethod evaluate1 ((self b-spline-plane-curve-mixin) (parameter real) &key (derivs 0))
  (declare (ignorable derivs))
  (let* ((knot-vector (the knot-vector))
	 (cpoints (the homogeneous-control-points))
	 (n (1- (length cpoints)))
	 (p (the degree))
	 (span (find-span n p parameter knot-vector))
	 (basis-funs (basis-funs span parameter p knot-vector)))
    (let ((Cw (vec3 0 0 0)))
      (loop for i from 0 to p
	    do (setq Cw (v+ Cw (v* (aref basis-funs i)
				   (aref cpoints (+ (- span p) i)))))
	    finally (return (vec2 (/ (vx Cw) (vz Cw)) (/ (vy Cw) (vz Cw))))))))


	   
				       

(defclass b-spline-plane-curve ()
  ()
  (:metaclass clui::protocol-class))

(defobject b-spline-plane-curve_a (b-spline-plane-curve-mixin entity-mixin)
  :inputs
  ((homogeneous-control-points
    (:validator (map 'vector #'(lambda (cp)
				 (setq cp (assure-double-vec3 cp))
				 (assert (not (zerop (vz cp))))
				 cp)
		     value)))
   (knot-vector
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value)))))

(defobject b-spline-plane-curve_b (b-spline-plane-curve-mixin entity-mixin)
  :inputs
  ((control-points
    (:validator (map 'vector #'(lambda (cp)
				 (assure-double-vec2 cp)) value)))
   (weights
    (:validator (map 'vector #'(lambda (w)
				 (assure-nonzero-double-float w)) value))
    (make-array (length (the control-points)) :initial-element 1.0d0))

   (knot-vector
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value)))))

(defmethod clui::compute-make-instance-arguments ((protocol b-spline-plane-curve)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'control-points provided-inputs)
		(member 'knot-vector provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'b-spline-plane-curve_b)) initargs))
	  ((and (member 'homogeneous-control-points provided-inputs)
		(member 'knot-vector provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'b-spline-plane-curve_a)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'b-spline-plane-curve ~S" initargs)))))

(defclass b-spline-space-curve ()
  ()
  (:metaclass clui::protocol-class))

(defobject b-spline-space-curve_a (b-spline-space-curve-mixin entity-mixin)
  :inputs
  ((homogeneous-control-points
    (:validator (map 'vector #'(lambda (cp)
				 (setq cp (assure-double-vec4 cp))
				 (assert (not (zerop (vw cp))))
				 cp)
		     value)))
   (knot-vector
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value)))))

(defobject b-spline-space-curve_b (b-spline-space-curve-mixin entity-mixin)
  :inputs
  ((control-points
    (:validator (map 'vector #'(lambda (cp)
				 (assure-double-vec3 cp)) value)))
   (weights
    (:validator (map 'vector #'(lambda (w)
				 (assure-nonzero-double-float w)) value))
    (make-array (length (the control-points)) :initial-element 1.0d0))

   (knot-vector
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value)))))

(defobject b-spline-3d-line-mixin (b-spline-space-curve-mixin)
  :inputs
  ((start (:validator (assure-double-vec3 value)))
   (end (:validator (assure-double-vec3 value))))

  :attributes
  ((control-points :uncached (vector (the start) (the end)))

   (weights :uncached (vector 1.0d0 1.0d0))
   
   (knot-vector :uncached (vector 0.0d0 0.0d0 1.0d0 1.0d0))))

(defobject b-spline-3d-line (b-spline-3d-line-mixin entity-mixin))

(defobject b-spline-3d-circle-mixin (b-spline-space-curve-mixin)
  :inputs
  ((center (:validator (assure-double-vec3 value)))
   (radius (:validator (assure-nonnegative-double-float value))))

  :attributes
  ((control-points
    :uncached (vector (v+ (the center) (vec3 (the radius) 0.0d0 0.0d0))
		      (v+ (the center) (vec3 (the radius) (* 2.0d0 (the radius)) 0.0d0))
		      (v+ (the center) (vec3 (- (the radius)) (* 2.0d0 (the radius)) 0.0d0))
		      (v+ (the center) (vec3 (- (the radius)) 0.0d0 0.0d0))
		      (v+ (the center) (vec3 (- (the radius)) (* -2.0d0 (the radius)) 0.0d0))
		      (v+ (the center) (vec3 (the radius) (* -2.0d0 (the radius)) 0.0d0))
		      (v+ (the center) (vec3 (the radius) 0.0d0 0.0d0))))

   (weights :uncached (map 'vector #'assure-double-float (list 1 1/3 1/3 1 1/3 1/3 1)))

   (knot-vector :uncached (map 'vector #'assure-double-float (list 0 0 0 0 1/2 1/2 1/2 1 1 1 1)))))

(defobject b-spline-3d-circle (b-spline-3d-circle-mixin entity-mixin))

(defmethod clui::compute-make-instance-arguments ((protocol b-spline-space-curve)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'control-points provided-inputs)
		(member 'knot-vector provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'b-spline-space-curve_b)) initargs))
	  ((and (member 'homogeneous-control-points provided-inputs)
		(member 'knot-vector provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'b-spline-space-curve_a)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'b-spline-space-curve ~S" initargs)))))

(defobject parametric-surface-mixin ())

(defobject b-spline-surface-mixin (b-spline-mixin parametric-surface-mixin)
  :attributes
  ((order-u :uncached (1+ (the degree-u)))
   
   (order-v :uncached (1+ (the degree-v)))
   
   (degree-u :uncached (let* ((m (1- (length (the knot-vector-u))))
			      (cpoints (the homogeneous-control-points))
			      (n (1- (first (array-dimensions cpoints)))))
			 ;; m = n + p + 1
			 (- m n 1)))
   
   (degree-v :uncached (let* ((m (1- (length (the knot-vector-v))))
			     (cpoints (the homogeneous-control-points))
			      (n (1- (second (array-dimensions cpoints)))))
			 ;; m = n + p + 1
			 (- m n 1)))
   
   (u-closed? :uncached nil)
   (v-closed? :uncached nil)
   (u-periodic? :uncached nil)
   (v-periodic? :uncached nil)
   (u-period :uncached nil)
   (v-period :uncached nil)
   
   (clamped-u? :uncached (let ((kv (the knot-vector-u))
			       (order (the order-u)))
			   (and (apply #'= (loop for i from 0 below order
						 collect (elt kv i)))
				(apply #'= (let ((m (1- (length kv))))
					     (loop for i from 0 below order
						   collect (elt kv (- m i))))))))
   
   (clamped-v? :uncached (let ((kv (the knot-vector-v))
			       (order (the order-v)))
			   (and (apply #'= (loop for i from 0 below order
						 collect (elt kv i)))
				(apply #'= (let ((m (1- (length kv))))
					     (loop for i from 0 below order
						   collect (elt kv (- m i))))))))
   
   (rational? :uncached (let* ((hcps (the homogeneous-control-points))
			       (dims (array-dimensions hcps)))
			  (not (loop for i from 0 below (first dims)
				     always (loop for j from 0 below (second dims)
						  always (= (vw (aref hcps i j)) 1.0d0))))))

   (domain-u :uncached (let ((kv (the knot-vector-u)))
			 (list (aref kv 0) (aref kv (1- (length kv))))))

   (domain-v :uncached (let ((kv (the knot-vector-v)))
			 (list (aref kv 0) (aref kv (1- (length kv))))))))

(defobject swept-surface-mixin (b-spline-surface-mixin)
  :inputs
  ((trajectory-curve
    (:validator (assert (typep value 'b-spline-space-curve-mixin)) value))
   (section-curve
    (:validator (assert (typep value 'b-spline-space-curve-mixin)) value)))

  :attributes
  ((homogeneous-control-points
    (let ((TR (the trajectory-curve homogeneous-control-points))
	  (Q (the section-curve homogeneous-control-points)))
      (make-array
       (list (length Q) (length TR))
       :initial-contents
       (loop for i from 0 below (length Q)
	     collect (loop for j from 0 below (length TR)
			   collect (let ((wij (* (vw (elt TR j)) (vw (elt Q i)))))
				     (vec4 (* wij (+ (/ (vx (elt TR j)) (vw (elt TR j))) (/ (vx (elt Q i)) (vw (elt Q i)))))
					   (* wij (+ (/ (vy (elt TR j)) (vw (elt TR j))) (/ (vy (elt Q i)) (vw (elt Q i)))))
					   (* wij (+ (/ (vz (elt TR j)) (vw (elt TR j))) (/ (vz (elt Q i)) (vw (elt Q i)))))
					   wij)))))))
   (knot-vector-u (the section-curve knot-vector))
   (knot-vector-v (the trajectory-curve knot-vector))))
		 
(defobject swept-surface (swept-surface-mixin entity-mixin))

(defmethod rm-draw-node ((self parametric-surface-mixin))
  (when (the visible?)
    (let* ((domain-u (the domain-u))
	   (domain-v (the domain-v))
	   (u-min (first domain-u))
	   (u-max (second domain-u))
	   (v-min (first domain-v))
	   (v-max (second domain-v))
	   (vertices ())
	   (indices ())
	   (resolution 100))
	
      (loop for i from 0 to resolution
	    with u-step = (/ (- u-max u-min) resolution)
	    with v-step = (/ (- v-max v-min) resolution)
	    do (let ((u (+ u-min (* i u-step))))
		 (loop for j from 0 to resolution
		       do (let ((v (+ v-min (* j v-step))))
			    (multiple-value-bind (point first-derivative) (evaluate2 self u v :derivs 1)
			      (let* ((du (first first-derivative))
				     (dv (first (last first-derivative)))
				     (normal (vc dv du)))
				(when (zerop (vsqrlength normal))
				  (setq normal (vunit (vec3 1 1 1))))
				(unless (zerop (vsqrlength normal))
				  (setq normal (vunit normal)))
				(push (krma::clampf (vx point)) vertices)
				(push (krma::clampf (vy point)) vertices)
				(push (krma::clampf (vz point)) vertices)
				(push (krma::clampf (vx normal)) vertices)
				(push (krma::clampf (vy normal)) vertices)
				(push (krma::clampf (vz normal)) vertices)
				))))))
    
      (loop for i from 0 below resolution
	    with k1 with k2
	    do (setq k1 (* i (1+ resolution))
		     k2 (+ k1 (1+ resolution)))
	       (loop for j from 1 to resolution

		     do (push k1 indices)
			(push k2 indices)
			(push (1+ k1) indices)
			
			
		     when (not (= i resolution))
		       do (push (1+ k1) indices)
			  (push k2 indices)
			  (push (1+ k2) indices)
			
			
		     do (incf k1)
			(incf k2)))

      (setq vertices (nreverse vertices))
      (setq indices (nreverse indices))

      (let ((scene (the krma:scene))
	    (oid (the object-id))
	    (color (krma::canonicalize-color #x00ffffff))
	    (group (or (the group) :default)))
	(krma::rm-dispatch-to-render-thread-with-handle (scene draw-data handle)
	  (let ((draw-list (krma::rm-draw-data-3d-triangle-list-with-normals-draw-list draw-data)))
	    (setf (gethash handle (krma::rm-draw-data-handle-hash-table draw-data))
		  (let* ((va (krma::draw-list-vertex-array draw-list))
			 (ia (krma::draw-list-index-array draw-list)))
		    (declare (type krma::foreign-adjustable-array ia va))
		    (let ((first-index (krma::foreign-array-fill-pointer ia))
			  (vtx-offset (krma::foreign-array-fill-pointer va))
			  (elem-count 0))
		      (krma::with-draw-list-transaction (%draw-list-draw-parametric-surface
							 draw-list first-index vtx-offset)
			(block krma::block
			  (loop for (x y z nx ny nz) on vertices by #'(lambda (list)
									(nthcdr 6 list))
				do (krma::standard-3d-vertex-with-normal-array-push-extend va oid x y z nx ny nz color))
			  (loop for index in indices
				do (krma::index-array-push-extend ia index)
				   (incf elem-count))
			  (let ((cmd (krma::make-standard-draw-indexed-cmd
				      draw-list
				      first-index elem-count vtx-offset
				      group nil nil krma::*white-texture* nil nil nil)))
			    (vector-push-extend cmd (krma::draw-list-cmd-vector draw-list))
			    cmd)))))))))))
  (values))

(defun assure-2d-array (value &key validator element-type error)
  (cond ((and (consp value)
	      (consp (first value)))
	 ;; list of lists
	 (let* ((n (length value))
		(m (length (first value)))
		(array (make-array (list n m) :element-type element-type)))
	   (loop for i from 0 below n
		 for row in value
		 do (loop for j from 0 below m
			  for elt in row
			  do (setf (aref array i j) (funcall validator elt))))
	   array))
		      
	((and (arrayp value)
	      (= 2 (array-rank value)))
	 ;; 2 dimensional array
	 (destructuring-bind (n m) (array-dimensions value)
	   (let ((array (make-array (list n m) :element-type element-type)))
	     (loop for i from 0 below n
		   do (loop for j from 0 below m
			    do (setf (aref array i j)
				     (funcall validator (aref value i j)))))
	     array)))
		      
	((and (consp value)
	      (arrayp (first value)))
	 ;; list of arrays
	 (let* ((n (length value))
		(m (length (first value)))
		(array (make-array (list n m) :element-type element-type)))
	   (loop for i from 0 below n
		 for row in value
		 do (loop for j from 0 below m
			  for elt across row
			  do (setf (aref array i j) (funcall validator elt))))
	   array))

	((and (arrayp value)
	      (= 1 (array-rank value))
	      (arrayp (aref value 0)))
	 ;; array of arrays
	 (let* ((n (length value))
		(m (length (aref value 0)))
		(array (make-array (list n m) :element-type element-type)))
	   (loop for i from 0 below n
		 for row across value
		 do (loop for j from 0 below m
			  for elt across row
			  do (setf (aref array i j) (funcall validator elt))))
	   array))
		      
	((and (arrayp value)
	      (consp (aref value 0)))
	 ;; array of lists
	 (let* ((n (length value))
		(m (length (aref value 0)))
		(array (make-array (list n m) :element-type element-type)))
	   (loop for i from 0 below n
		 do (loop for j from 0 below m
			  for elt in (aref value i)
			  do (setf (aref array i j)
				   (funcall validator elt))))
	   array))
		      
	(t (funcall error value))))
  
(defclass b-spline-surface ()
  ()
  (:metaclass clui::protocol-class))

(defobject b-spline-surface_a (b-spline-surface-mixin entity-mixin)
  :inputs
  ((control-points
    (:validator (assure-2d-array value :validator #'assure-double-vec3
				       :element-type 'vec3
				       :error #'(lambda (value)
						  (error "Invalid control-points: ~S" value)))))
		       
		       
   (weights
    (:validator (assure-2d-array value :validator #'assure-nonzero-double-float
				       :element-type 'double-float
				       :error #'(lambda (value)
						  (error "Invalid weights: ~S" value)))))

   (knot-vector-u
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value)))
   (knot-vector-v
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value))))
    
  :attributes
  ((homogeneous-control-points
    (let ((cpoints (the control-points))
	  (weights (the weights)))
      (destructuring-bind (n m) (array-dimensions cpoints)
	(let ((array (make-array (list n m) :element-type 'vec4 :initial-element (vec4 0 0 0 0))))
	  (loop for i from 0 below n
		do (loop for j from 0 below m
			 do (let ((cpt (aref cpoints i j))
				  (w (aref weights i j)))
			      (setf (aref array i j) (vec4 (* w (vx cpt))
							   (* w (vy cpt))
							   (* w (vz cpt))
							   w)))))
	  array))))))

(defobject b-spline-surface_b (b-spline-surface-mixin entity-mixin)
  :inputs
  ((homogeneous-control-points
    (:validator (assure-2d-array value
				 :validator #'(lambda (elt)
						(setq elt (assure-double-vec4 elt))
						(assert (not (zerop (vw elt))))
						elt)
				 :element-type 'vec4
				 :error #'(lambda (value)
					    (error "Invalid homogeneous-control-points: ~S" value)))))
   (knot-vector-u
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value)))
   (knot-vector-v
    (:validator (map 'vector #'(lambda (elt)
				 (assure-double-float elt)) value))))

  :attributes
  ((control-points
    (let ((cpoints (the homogeneous-control-points)))
      (destructuring-bind (n m) (array-dimensions cpoints)
	(let ((array (make-array (list n m) :element-type 'vec3
					    :initial-element (load-time-value (vec3 0 0 0)))))
	  (loop for i from 0 below n
		do (loop for j from 0 below m
			 do (let ((cpt (aref cpoints i j)))
			      (setf (aref array i j) (vec3 (/ (vx cpt) (vw cpt))
							   (/ (vy cpt) (vw cpt))
							   (/ (vz cpt) (vw cpt)))))))
	  array))))

   (weights
    (let ((cpoints (the homogeneous-control-points)))
      (destructuring-bind (n m) (array-dimensions cpoints)
	(let ((array (make-array (list n m) :element-type 'double-float)))
	  (loop for i from 0 below n
		do (loop for j from 0 below m
			 do (let ((cpt (aref cpoints i j)))
			      (setf (aref array i j) (vw cpt)))))
	  array))))))

(defmethod clui::compute-make-instance-arguments ((protocol b-spline-surface)
						  &rest initargs
						  &key &allow-other-keys)
  (let ((provided-inputs (getf initargs 'adhoc::provided-inputs (plist-keys initargs))))
    (cond ((and (member 'control-points provided-inputs)
		(member 'knot-vector-u provided-inputs)
		(member 'knot-vector-v provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'b-spline-surface_b)) initargs))
	  ((and (member 'homogeneous-control-points provided-inputs)
		(member 'knot-vector-u provided-inputs)
		(member 'knot-vector-v provided-inputs))
	   (remf initargs 'adhoc::provided-inputs)
	   (list* (load-time-value (find-class 'b-spline-surface_a)) initargs))
	  (t (remf initargs 'adhoc::provided-inputs)
	     (error "No suitable concrete class found for args 'b-spline-surface ~S" initargs)))))

(defun surface-derivs-alg1 (n p knot-vector-u m q knot-vector-v control-points u v d)
  ;; Algorithm A3.6, The Nurbs Book, 2nd edition, p. 111
  (setq u (coerce u 'double-float))
  (setq v (coerce v 'double-float))
  (let* ((du (min d p))
	 (dv (min d q))
	 (uspan (find-span n p u knot-vector-u))
	 (vspan (find-span m q v knot-vector-v))
	 (Nu (ders-basis-funs uspan u p du knot-vector-u))
	 (Nv (ders-basis-funs vspan v q dv knot-vector-v))
	 (SKL (make-array (list (1+ d) (1+ d)) :initial-element (vec4 0 0 0 0)))
	 (temp (make-array (1+ q) :initial-element (vec4 0 0 0 0))))
    (loop for k from 0 to du
	  do (loop for s from 0 to q
		   do (setf (aref temp s) (vec4 0 0 0 0))
		      (loop for r from 0 to p
			    do (setf (aref temp s) (v+ (aref temp s)
						       (v* (aref Nu k r) (aref control-points (+ (- uspan p) r) (+ (- vspan q) s)))))))
	     (loop for l from 0 to (min (- d k) dv)
		   do (setf (aref SKL k l) (vec4 0 0 0 0))
		      (loop for s from 0 to q
			    do (setf (aref SKL k l) (v+ (aref SKL k l) (v* (aref Nv l s) (aref temp s)))))))
    SKL))

(defun rat-surface-derivs (surface u v d)
  (let* ((cpoints (send surface homogeneous-control-points))
	 (n (1- (first (array-dimensions cpoints))))
	 (m (1- (second (array-dimensions cpoints))))
	 (kv-u (send surface knot-vector-u))
	 (kv-v (send surface knot-vector-v))
	 (d-u (send surface degree-u))
	 (d-v (send surface degree-v))
	 (ders (surface-derivs-alg1 n d-u kv-u
				    m d-v kv-v
				    cpoints
				    u v d))
	 (SKL (make-array (list (1+ d) (1+ d))
			  :initial-element nil)))
    ;; Algorithm A4.4, The Nurbs Book, 2nd edition, p. 137-138 
    (loop for k from 0 to d
	  do (loop for l from 0 to (- d k)
		   do (let ((v0 (vec3 (vx (aref ders k l)) (vy (aref ders k l)) (vz (aref ders k l)))))
			(loop for j from 1 to l
			      do (setf v0 (v- v0 (v* (* (binomial-coefficient l j) (vw (aref ders 0 j))) (aref SKL k (- l j))))))
			(loop for i from 1 to k
			      do (setf v0 (v- v0 (v* (* (binomial-coefficient k i) (vw (aref ders i 0))) (aref SKL (- k i) l))))
				 (let ((v2 (vec3 0 0 0)))
				   (loop for j from 1 to l
					 do (setf v2 (v+ v2 (v* (* (binomial-coefficient l j) (vw (aref ders i j))) (aref SKL (- k i) (- l j))))))
				   (setf v0 (v- v0 (v* (binomial-coefficient k i) v2)))))
			(setf (aref SKL k l) (v/ v0 (vw (aref ders 0 0)))))))
    SKL))

(defmethod evaluate2 ((self b-spline-surface-mixin) (u real) (v real) &key (derivs 0))

  (if (> derivs 0)
      
      (let* ((d (min derivs (max (the degree-u) (the degree-v))))
	     (results (rat-surface-derivs self u v d)))
	
	(values-list (append (list (aref results 0 0))
			     (loop for i from 1 to d
				   collect
				   (append (loop for j from 0 to i
						 collect (aref results j i))
					   (loop for j from (1- i) downto 0
						 collect (aref results i j)))))))
      
      (let* ((knot-vector-u (the knot-vector-u))
	     (knot-vector-v (the knot-vector-v))
	     (Pw (the homogeneous-control-points))
	     (dims (array-dimensions Pw))
	     (n (1- (first dims)))
	     (m (1- (second dims)))
	     (p (the degree-u))
	     (q (the degree-v))
	     (uspan (find-span n p u knot-vector-u))
	     (uspan-p (- uspan p))
	     (Nu (basis-funs uspan u p knot-vector-u))
	     (vspan (find-span m q v knot-vector-v))
	     (vspan-q (- vspan q))
	     (Nv (basis-funs vspan v q knot-vector-v))
	     (temp (make-array (1+ q)
			       :element-type 'vec4
			       :initial-element (load-time-value (vec4 0 0 0 0))))
	     (Sw (load-time-value (vec4 0 0 0 0))))
	
	(loop for l from 0 to q
	      do (loop for k from 0 to p
		       do (setf (aref temp l)
				(v+
				 (cl:the vec4 (aref temp l))
				 (v*
				  (aref Nu k)
				  (cl:the vec4 (aref Pw (+ uspan-p k) (+ vspan-q l)))))))
	      finally (loop for l from 0 to q
			    do (setq Sw (v+
					 (cl:the vec4 Sw)
					 (v* (aref Nv l)
					     (cl:the vec4 (aref temp l))))))
		      (return (safe-euclid Sw))))))

(defobject solid-mixin ()
  :inputs
  ((color #xffff)
   (material krma::*default-material*)
   (show-edges? t))

  :attributes
  ((oc-gprops (let ((GProps (oc::_wrap_new_GProp_GProps__SWIG_0)))
		(oc::_wrap_BRepGProp_VolumeProperties__SWIG_0 (oc::ff-pointer (the body)) GProps nil)
		GProps))
   (centroid :uncached (let ((gp-pnt (oc::_wrap_GProp_GProps_CentreOfMass (the oc-gprops))))
			 (prog1 (vec3 (cffi:mem-aref gp-pnt :double 0)
				      (cffi:mem-aref gp-pnt :double 1)
				      (cffi:mem-aref gp-pnt :double 2))
			   (oc::_wrap_delete_gp_pnt gp-pnt))))
   (mass :uncached (oc::_wrap_GProp_GProps_Mass (the oc-gprops)))))

(defmethod centroidish ((thing oc::topods-face))
  (let ((gprops (oc::_wrap_new_GProp_GProps__SWIG_0)))
    (oc::_wrap_BRepGProp_SurfaceProperties__SWIG_0 (oc::ff-pointer thing) gprops)
    (let ((gp-pnt (oc::_wrap_GProp_GProps_CentreOfMass gprops)))
      (prog1 (vec3 (cffi:mem-aref gp-pnt :double 0)
		   (cffi:mem-aref gp-pnt :double 1)
		   (cffi:mem-aref gp-pnt :double 2))
	(oc::_wrap_delete_gp_pnt gp-pnt)))))

(defmethod area ((thing oc::topods-face))
  (let ((gprops (oc::_wrap_new_GProp_GProps__SWIG_0)))
    (oc::_wrap_BRepGProp_SurfaceProperties__SWIG_0 (oc::ff-pointer thing) gprops)
    (oc::_wrap_GProp_GProps_Mass gprops)))

(defmethod centroidish ((thing oc::topods-compound))
  (let ((gprops (oc::_wrap_new_GProp_GProps__SWIG_0)))
    (oc::_wrap_BRepGProp_VolumeProperties__SWIG_0 (oc::ff-pointer thing) gprops nil)
    (let ((gp-pnt (oc::_wrap_GProp_GProps_CentreOfMass gprops)))
      (prog1 (vec3 (cffi:mem-aref gp-pnt :double 0)
		   (cffi:mem-aref gp-pnt :double 1)
		   (cffi:mem-aref gp-pnt :double 2))
	(oc::_wrap_delete_gp_pnt gp-pnt)))))

(defmethod centroidish ((thing oc::topods-solid))
  (let ((gprops (oc::_wrap_new_GProp_GProps__SWIG_0)))
    (oc::_wrap_BRepGProp_VolumeProperties__SWIG_0 (oc::ff-pointer thing) gprops nil)
    (let ((gp-pnt (oc::_wrap_GProp_GProps_CentreOfMass gprops)))
      (prog1 (vec3 (cffi:mem-aref gp-pnt :double 0)
		   (cffi:mem-aref gp-pnt :double 1)
		   (cffi:mem-aref gp-pnt :double 2))
	(oc::_wrap_delete_gp_pnt gp-pnt)))))

(defmethod centroidish ((thing oc::topods-shell))
  (let ((gprops (oc::_wrap_new_GProp_GProps__SWIG_0)))
    (oc::_wrap_BRepGProp_VolumeProperties__SWIG_0 (oc::ff-pointer thing) gprops nil)
    (let ((gp-pnt (oc::_wrap_GProp_GProps_CentreOfMass gprops)))
      (prog1 (vec3 (cffi:mem-aref gp-pnt :double 0)
		   (cffi:mem-aref gp-pnt :double 1)
		   (cffi:mem-aref gp-pnt :double 2))
	(oc::_wrap_delete_gp_pnt gp-pnt)))))

(defun selection-set-centroid (application-frame)
  (let* ((ss (climish::selection-set application-frame))
	 (size (fset:size ss))
	 (sum (vec3 0 0 0)))
    (when (zerop size)
      (return-from selection-set-centroid sum))
    (fset:do-map (entity point ss)
      (declare (ignore point))
      (setq sum (v+ sum (if (typep entity 'node-mixin)
			    (transform-coordinate-from-to (centroidish entity) entity nil)
			    (centroidish entity)))))
    (v/ sum size)))
    
    
  


(defmethod rm-draw-node ((self solid-mixin))
  (when (the visible?)
    (rm-draw-solid self
		   (the krma:scene)
		   (the body)
		   :group (the group)
		   :color (the color)
		   :material (the material)
		   :show-edges? (the show-edges?)))
  (values))

(defmethod rm-erase-node ((self solid-mixin))
  (let ((basket (adhoc::slot-value-internal self 'body)))
    ;; don't regenerate body if it's already gone or was never there.
    (unless (eq basket adhoc::+slot-unbound+)
      (when (slot-boundp basket 'adhoc::value)
	(rm-erase-solid self
			(the krma:scene)
			(slot-value basket 'adhoc::value)))))
  (values))

(defmethod rm-erase-solid ((self solid-mixin)
			   (scene krma:krma-essential-scene-mixin)
			   (shape oc::topods-compound))
  (mapcar #'(lambda (shape)
	      (rm-erase-solid self scene shape))
	  (topods-children shape))
  (values))

(defmethod rm-erase-solid ((self solid-mixin)
			   (scene krma:krma-essential-scene-mixin)
			   (shape oc::topods-solid))
  (mapcar #'(lambda (shape)
	      (rm-erase-solid self scene shape))
	  (topods-children shape))
  (values))

(defmethod rm-erase-solid ((self solid-mixin)
			   (scene krma:krma-essential-scene-mixin)
			   (shape oc::topods-shell))
  (mapcar #'(lambda (shape)
	      (rm-erase-solid self scene shape))
	  (topods-children shape))
  (values))


(defmethod rm-erase-solid ((self solid-mixin)
			   (scene krma:krma-essential-scene-mixin)
			   (shape oc::topods-face))
  (krma:delete-primitives scene (send shape climish::primitive-handles))
  (forget-object shape)
  (setf (send shape climish::primitive-handles) nil)
  (setf (send shape triangulation) nil)
  (values))

(defmethod rm-retransform-node :before ((self solid-mixin))
  (rm-retransform-solid self (the krma:scene) (the shape)))

(defmethod rm-retransform-solid ((self solid-mixin) (scene krma:krma-essential-scene-mixin)
				 (shape oc::topods-compound))
  ;; transform solid here
  )

(defmethod rm-retransform-solid ((self solid-mixin) (scene krma:krma-essential-scene-mixin)
				 (shape oc::topods-solid))
  ;; transform solid here
  )

(defmethod rm-retransform-solid ((self solid-mixin) (scene krma:krma-essential-scene-mixin)
				 (shape oc::topods-shell))
  ;; transform solid here
  )

(defmethod rm-retransform-solid ((self solid-mixin) (scene krma:krma-essential-scene-mixin)
				 (shape oc::topods-face))
  ;; transform solid here
  )

(defmethod rm-retransform-solid ((self solid-mixin) (scene krma:krma-essential-scene-mixin)
				 (shape oc::topods-edge))
  ;; transform solid here
  )

(defmethod rm-retransform-solid ((self solid-mixin) (scene krma:krma-essential-scene-mixin)
				 (shape oc::topods-vertex))
  ;; transform solid here
  )


  


(defun topods-children (topods-shape)
  (when topods-shape
    (let ((iterator (make-instance 'oc::TopoDS-Iterator :S topods-shape)))
      (loop while (oc::more-p iterator)
            collect (prog1 (oc::value iterator)
                      (oc:next iterator))))))

(defmethod rm-draw-solid ((self solid-mixin)
			  (scene krma:krma-essential-scene-mixin)
			  (shape oc::topods-compound)
			  &key (group :default)
			    (color #xffffff)
			    (material krma::*default-material*)
			    (show-edges? t)
			    (mesh (make-instance 'oc::BRep-Mesh-Incremental-Mesh
						 :theShape shape
						 :theLinDeflection 2.0d-2
						 :theAngDeflection 1.0d-1
						 :isRelative t
						 :isInParallel t))
			  &allow-other-keys)
  (mapcar #'(lambda (shape)
	      (rm-draw-solid self scene shape
			     :group group
			     :color color
			     :material material
			     :show-edges? show-edges?
			     :mesh mesh))
	  (topods-children shape))
  (values))

(defmethod rm-draw-solid ((self solid-mixin)
			  (scene krma:krma-essential-scene-mixin)
			  (shape oc::topods-solid)
			  &key (group :default)
			    (color #xffffff)
			    (material krma::*default-material*)
			    (show-edges? t)
			    (mesh (make-instance 'oc::BRep-Mesh-Incremental-Mesh
						 :theShape shape
						 :theLinDeflection 2.0d-2
						 :theAngDeflection 1.0d-1
						 :isRelative t
						 :isInParallel t))
			  &allow-other-keys)
  (mapcar #'(lambda (shape)
	      (rm-draw-solid self scene shape
			     :group group
			     :color color
			     :material material
			     :show-edges? show-edges?
			     :mesh mesh))
	  (topods-children shape))
  (values))

(defmethod rm-draw-solid ((self solid-mixin)
			  (scene krma:krma-essential-scene-mixin)
			  (shape oc::topods-shell)
			  &key (group :default)
			    (color #xffffff)
			    (material krma::*default-material*)
			    (show-edges? t)
			    (mesh (make-instance 'oc::BRep-Mesh-Incremental-Mesh
						 :theShape shape
						 :theLinDeflection 2.0d-2
						 :theAngDeflection 1.0d-1
						 :isRelative t
						 :isInParallel t))
			  &allow-other-keys)
  (let ((faces (topods-children shape)))
    (mapcar #'(lambda (shape)
		(rm-draw-solid self scene shape
			       :group group
			       :color color
			       :material material
			       :mesh mesh))
	    faces)
    (when show-edges?
      (let ((face-edges (mapcar #'(lambda (face)
				    (let ((wires (topods-children face)))
				      (apply #'append (mapcar #'topods-children wires))))
				faces)))
	(let ((exp (make-instance 'oc::top-exp-explorer
				  :S shape
				  :ToFind :topabs_edge)))
	  (loop while (oc::more-p exp)
		do (let* ((edge (oc::current exp))
			  (face (nth (position edge face-edges :test #'member) faces)))
		     (rm-draw-solid self scene edge
				    :face face
				    :group group
				    :color #x000000ff
				    :mesh mesh))
		   (oc::next exp))))))			      
  (values))

(defmethod rm-draw-solid ((self solid-mixin)
			  (scene krma:krma-essential-scene-mixin)
			  (shape oc::topods-face)
			  &key (group :default)
			    (color #xffffff)
			    (material krma::*default-material*)
			  &allow-other-keys)
  (oc-face-triangulation shape)
  (push (krma::scene-add-diffuse-opencascade-face scene group nil color material shape)
	(send shape climish::primitive-handles))
  (values))

(defmethod rm-draw-solid ((self solid-mixin)
			  (scene krma:krma-essential-scene-mixin)
			  (shape oc::topods-edge)
			  &key (group :default)
			    (color #x000000ff)
			    face
			  &allow-other-keys)
  (when face
    (push (krma::scene-add-opencascade-edge scene group nil color shape face)
	  (send shape climish::primitive-handles)))
  (values))

krma::(defun scene-add-opencascade-edge (scene group model-matrix color edge face)
	(declare (type krma-essential-scene-mixin scene))
	(declare (type (and atom t) group))
	(declare (type oc::topods-edge edge))
	(declare (type oc::topods-face face))
	(declare (type (or mat4 null) model-matrix))
	(let ((vertices (let* ((hpt (rgn-geo::oc-face-triangulation face))
			       (hpolygon (oc::_wrap_BRep_Tool_PolygonOnTriangulation__SWIG_0
					  (oc::ff-pointer edge)
					  hpt
					  (oc::_wrap_new_TopLoc_Location__SWIG_0)))
			       (polygon (cffi::mem-aref hpolygon :pointer))
			       (nodes (oc::_wrap_Poly_PolygonOnTriangulation_Nodes polygon))
			       (lower (OC::_WRAP_TCOLSTD_ARRAY1OFINTEGER_LOWER nodes))
			       (upper (OC::_WRAP_TCOLSTD_ARRAY1OFINTEGER_Upper nodes))
			       (pt (cffi::mem-aref hpt :pointer))
			       (coordinates (oc::poly_triangulation_node_array_pointer pt))
			       (clower (oc::poly_triangulation_node_array_lower pt)))
			  (loop for i from lower to upper
				nconc (let* ((index (oc::_wrap_TColStd_Array1OfInteger_value nodes i))
					     (offset (* 3 (- index clower))))
					(list
					 (clampf (cffi:mem-aref coordinates :double offset))
					 (clampf (cffi:mem-aref coordinates :double (1+ offset)))
					 (clampf (cffi:mem-aref coordinates :double (+ 2 offset))))))))
		
	      (ub32-oid (oc::_wrap_topods_shape_hashcode (oc::ff-pointer edge)
							 (load-time-value (1- (expt 2 31))))))
	  (scene-add-filled-3d-instanced-tube-primitive
	   scene group model-matrix nil 2 color vertices ub32-oid)))


(in-package :krma)

(defun scene-add-diffuse-opencascade-face (scene group model-matrix color material face)
	(declare (type krma-essential-scene-mixin scene))
	(declare (type (and atom t) group))
	(declare (type oc::topods-face face))
	(declare (type (or mat4 null) model-matrix))
	(declare (type (or standard-material null) material))
	(setq color (canonicalize-color color))
	(rgn-geo::oc-face-triangulation face)
	(let ((ub32-oid (oc::_wrap_topods_shape_hashcode (oc::ff-pointer face)
							 (load-time-value (1- (expt 2 31)))))
	      (orientation (OC::_WRAP_TOPODS_SHAPE_ORIENTATION__SWIG_0 (oc::ff-pointer face))))
	  (setf (krma::object-from-id ub32-oid) face)
	  (rm-dispatch-to-render-thread-with-handle (scene draw-data handle)
	    (%draw-data-add-diffuse-opencascade-face
	     draw-data handle face group (when model-matrix (mcopy model-matrix))
	     color material orientation ub32-oid))))

(defun %draw-data-add-diffuse-opencascade-face
    (draw-data handle topods-face atom-group model-mtx ub32-color material orientation ub32-oid)
  (declare (type retained-mode-draw-data draw-data))
  (setf (gethash handle (rm-draw-data-handle-hash-table draw-data))
	(krma::with-next-draw-list (draw-list
				    cmd-vector
				    (rm-draw-data-3d-triangle-list-with-normals-draw-list draw-data)
				    (make-instance (load-time-value (find-class '3d-vertex-with-normal-draw-list))))
	  (krma::%draw-list-add-diffuse-opencascade-face
	   draw-list cmd-vector topods-face ub32-oid atom-group model-mtx ub32-color material orientation))))

(defun %draw-list-add-diffuse-opencascade-face
	  (3d-draw-list cmd-vector topods-face ub32-oid atom-group model-mtx ub32-color material orientation)
	(declare (type 3d-vertex-with-normal-draw-list-mixin 3d-draw-list))
	(declare (type (unsigned-byte 32) ub32-oid ub32-color))
	(let* ((index-array (draw-list-index-array 3d-draw-list))
	       (vertex-array (draw-list-vertex-array 3d-draw-list))
	       (vtx-offset (foreign-array-fill-pointer vertex-array))
	       (first-index (foreign-array-fill-pointer index-array)))
	  (with-draw-list-transaction (%draw-list-add-diffuse-opencascade-face
				       3d-draw-list first-index vtx-offset)
	    (let* ((hpt (rgn-geo::oc-face-triangulation topods-face))
		   (pt (cffi:mem-aref hpt :pointer))
		   (n-tris (oc::poly_triangulation_nb_triangles pt))
		   (n-nodes (oc::poly_triangulation_nb_nodes pt))
		   (tri-array (oc::poly_triangulation_triangles_array_pointer pt))
		   (node-array (oc::poly_triangulation_node_array_pointer pt))
		   (lower (oc::poly_triangulation_node_array_lower pt))
		   (normals-array (oc::poly_triangulation_normal_array_pointer pt)))
	      (if (eq orientation :topabs_reversed)
		  (loop repeat n-nodes
			for j from 0
			do (let* ((i (* 3 j))
				  (1+i (1+ i))
				  (2+i (+ 2 i)))
			     (standard-3d-vertex-with-normal-array-push-extend
			      vertex-array ub32-oid
			      (clampf (cffi:mem-aref node-array :double i))
			      (clampf (cffi:mem-aref node-array :double 1+i))
			      (clampf (cffi:mem-aref node-array :double 2+i))
			      (- (cffi:mem-aref normals-array :float i))
			      (- (cffi:mem-aref normals-array :float 1+i))
			      (- (cffi:mem-aref normals-array :float 2+i))
			      ub32-color)))
		  (loop repeat n-nodes
			for j from 0
			do (let* ((i (* 3 j))
				  (1+i (1+ i))
				  (2+i (+ 2 i)))
			     (standard-3d-vertex-with-normal-array-push-extend
			      vertex-array ub32-oid
			      (clampf (cffi:mem-aref node-array :double i))
			      (clampf (cffi:mem-aref node-array :double 1+i))
			      (clampf (cffi:mem-aref node-array :double 2+i))
			      (cffi:mem-aref normals-array :float i)
			      (cffi:mem-aref normals-array :float 1+i)
			      (cffi:mem-aref normals-array :float 2+i)
			      ub32-color))))
	      (loop repeat n-tris
		    for j from 0
		    do (let* ((i (* 3 j))
			      (1+i (1+ i))
			      (2+i (+ 2 i)))
			 (index-array-push-extend index-array (- (cffi:mem-aref tri-array :int i) lower))
			 (index-array-push-extend index-array (- (cffi:mem-aref tri-array :int 1+i) lower))
			 (index-array-push-extend index-array (- (cffi:mem-aref tri-array :int 2+i) lower))))
	      (let ((cmd (make-standard-draw-indexed-cmd
			  3d-draw-list
			  first-index (* 3 n-tris) vtx-offset
			  atom-group model-mtx
			  nil *white-texture* nil nil material)))
		(vector-push-extend cmd cmd-vector)
		cmd)))))
  
(in-package :rgn-geo)

(defun make-opencascade-wire (oc-edges)
  (let ((mw (make-instance 'oc::brep-builder-api-make-wire)))
    (loop for edge in oc-edges
	  do (oc::add mw edge)
	  finally (return (when (oc::done? mw) (oc::wire mw))))))

(defun make-opencascade-edge (oc-curve)
  (oc::edge (make-instance 'oc::BRep-Builder-API-make-edge :L oc-curve)))

(defclass oc::topods-face (oc::topods-shape)
  ((triangulation :initform nil)
   (climish::primitive-handles :initform nil)))

(defclass oc::topods-edge (oc::topods-shape)
  ((climish::primitive-handles :initform nil)))

(defun oc-face-triangulation (topods-face)
  (let ((triangulation (slot-value topods-face 'triangulation)))
    (or triangulation
	(setf (slot-value topods-face 'triangulation)
	      (let ((tri
		      (oc::_wrap_BRep_Tool_Triangulation (oc::ff-pointer topods-face)
							 (oc::_wrap_new_TopLoc_Location__SWIG_0))))
		(oc::_wrap_poly_computenormals tri)
		tri)))))

(defobject profile (solid-mixin entity-mixin)
  :inputs
  ((outer-loop (:validator (map 'list #'(lambda (elt)
					  (assert (typep elt 'curve-mixin))
					  elt)
				value)))
   (inner-loops (:validator (map 'list #'(lambda (list)
					   (map 'list #'(lambda (elt)
							  (assert (typep elt 'curve-mixin))
							  elt)
						list))
				 value))
		nil))

  :attributes
  ((oc-outer-loop (make-opencascade-wire
		   (mapcar #'make-opencascade-edge (mapcar #'to-opencascade (the outer-loop)))))
   (oc-inner-loops (mapcar #'(lambda (list)
			       (make-opencascade-wire
				(mapcar #'make-opencascade-edge
					(mapcar #'to-opencascade list))))
			   (the inner-loops)))
   (body (let* ((f (oc::face (make-instance 'oc::BRep-Builder-API-Make-Face
					    :W (the oc-outer-loop)
					    :OnlyPlane t)))
		(mf (make-instance 'oc::BRep-Builder-API-Make-Face :F f)))
	   (loop for w in (the oc-inner-loops)
		 do (oc::Add mf w)
		 finally (return (let ((face (oc::face mf)))
				   (make-instance 'oc::BRep-Mesh-Incremental-Mesh
						  :theShape face
						  :theLinDeflection 2.0d-2
						  :theAngDeflection 1.0d-1
						  :isRelative t
						  :isInParallel t)
				   (oc-face-triangulation face)
				   face)))))))

(defobject extrusion (solid-mixin entity-mixin)
  :inputs
  ((profile (:validator (assert (typep value 'profile)) value))
   (vector (:validator (assure-double-vec3 value))))

  :attributes
  ((body (oc::shape (make-instance 'oc::BRep-Prim-API-Make-Prism
				   :Copy t
				   :baseShape (the profile body)
				   :Canonize t
				   :extrudeDirection (let ((v (the vector)))
						       (gp:vec (vx v) (vy v) (vz v))))))))

(defobject subtraction (solid-mixin entity-mixin)
  :inputs
  ((objects (:validator (map 'list #'(lambda (elt)
				       (assert (typep elt 'solid-mixin))
				       elt)
			     value)))
   (tools (:validator (map 'list #'(lambda (elt)
				     (assert (typep elt 'solid-mixin))
				     elt)
			   value))))
  :attributes
  ((oc-objects (let ((list (make-instance 'oc::top-tools-list-of-shape)))
		 (loop for solid in (the objects)
		       do (oc::list-append list (send solid body))
		       finally (return list))))
   (oc-tools (let ((list (make-instance 'oc::top-tools-list-of-shape)))
	       (loop for solid in (the tools)
		     do (oc::list-append list (send solid body))
		     finally (return list))))
   (builder (oc::_wrap_new_BRepAlgoAPI_Cut__SWIG_0))
   (body (let ((builder (the builder)))
	    (oc::_wrap_BRepAlgoAPI_Algo_SetRunParallel builder t)
	    (oc::_wrap_BRepAlgoAPI_BuilderAlgo_SetFuzzyValue builder 2.1d-5)
	    (oc::_wrap_BRepAlgoAPI_BuilderAlgo_SetArguments builder (oc::ff-pointer (the oc-objects)))
	    (oc::_wrap_BRepAlgoAPI_BooleanOperation_SetTools builder (oc::ff-pointer (the oc-tools)))
	    (oc::_wrap_BRepAlgoAPI_BuilderAlgo_Build builder)
	    (oc::with-topods-shape (oc::_wrap_BRepAlgoAPI_Algo_Shape builder))))))
	    
	    
						    
  
(defobject block-mixin (group-owner-mixin node-mixin))

(defmethod rm-erase-node ((self block-mixin))
  (krma:delete-group (the krma:scene) (the group)))

(defobject cylinder-mixin (block-mixin))

(defobject solid-primitive-mixin (solid-mixin entity-mixin)
  :inputs ((body :visual)))

(defobject cylinder_b (cylinder-mixin)
  :inputs
  ((radius :visual 1.0d0)
   (height :visual 1.0d0)
   (center-on :visual (vec3 0 0 0))
   (display :visual :wireframe))

  :attributes
  ((origin (v+ (the center-on) (safe-euclid (m* (the rotation-matrix) (vec4 0 0 (- (half (the height))) 1.0d0)))))
   (oc-normal (gp:dir (vx (the :bottom-circle normal)) (vy (the :bottom-circle normal)) (vz (the :bottom-circle normal))))
   (oc-center (gp:pnt (vx (the :bottom-circle center)) (vy (the :bottom-circle center)) (vz (the :bottom-circle center))))
   (oc-ax2 (gp:ax2 :p (the oc-center) :n (the oc-normal)))
   (oc-solid (oc::shape (make-instance 'oc::brep-prim-api-make-cylinder
				      :axes (the oc-ax2)
				      :h (coerce (the height) 'double-float)
				      :r (coerce (the radius) 'double-float)))))
  
  :components
  ((:bottom-circle :type '3d-circle_a
		   center (vec3 0 0 0)
		   normal (vec3 0 0 1)
		   radius (the radius)
		   visible? (eq (the display) :wireframe))
   (:top-circle :type '3d-circle_a
		center (vec3 (vx (the :bottom-circle center))
			     (vy (the :bottom-circle center))
			     (the height))
		normal (the :bottom-circle normal)
		radius (the :bottom-circle radius)
		visible? (eq (the display) :wireframe))
   (line-1 :type 'line
	   start (v+ (the :bottom-circle center) (v* (the :bottom-circle radius) (vec3 1 0 0)))
	   end (v+ (the :top-circle center) (v* (the :top-circle radius) (vec3 1 0 0)))
	   visible? (eq (the display) :wireframe))
   (line-2 :type 'line
	   start (v+ (the :bottom-circle center) (v* (the :bottom-circle radius) (vec3 0 1 0)))
	   end (v+ (the :top-circle center) (v* (the :top-circle radius) (vec3 0 1 0)))
	   visible? (eq (the display) :wireframe))
   (line-3 :type 'line
	   start (v+ (the :bottom-circle center) (v* (the :bottom-circle radius) (vec3 -1 0 0)))
	   end (v+ (the :top-circle center) (v* (the :top-circle radius) (vec3 -1 0 0)))
	   visible? (eq (the display) :wireframe))
   (line-4 :type 'line
	   start (v+ (the :bottom-circle center) (v* (the :bottom-circle radius) (vec3 0 -1 0)))
	   end (v+ (the :top-circle center) (v* (the :top-circle radius) (vec3 0 -1 0)))
	   visible? (eq (the display) :wireframe))
   
   (:solid :type 'solid-primitive-mixin
	   body (the oc-solid)
	   visible? (eq (the display) :shaded))))

(defobject hollow-cylinder_b (cylinder_b)
  :inputs ((thickness 0.1d0))
  :attributes
  ((oc-solid-2 (oc::shape (make-instance 'oc::brep-prim-api-make-cylinder
				      :axes (the oc-ax2)
				      :h (coerce (the height) 'double-float)
				      :r (coerce (- (the radius) (the thickness)) 'double-float)))))

  :components
  ((:bottom-circle-2 :type '3d-circle_a
		   center (vec3 0 0 0)
		   normal (vec3 0 0 1)
		   radius (- (the radius) (the thickness))
		   visible? (eq (the display) :wireframe))
   (:top-circle-2 :type '3d-circle_a
		center (vec3 (vx (the :bottom-circle-2 center))
			     (vy (the :bottom-circle-2 center))
			     (the height))
		normal (the :bottom-circle-2 normal)
		radius (the :bottom-circle-2 radius)
		visible? (eq (the display) :wireframe))
   (line-1-2 :type 'line
	   start (v+ (the :bottom-circle-2 center) (v* (the :bottom-circle-2 radius) (vec3 1 0 0)))
	   end (v+ (the :top-circle-2 center) (v* (the :top-circle-2 radius) (vec3 1 0 0)))
	   visible? (eq (the display) :wireframe))
   (line-2-2 :type 'line
	   start (v+ (the :bottom-circle-2 center) (v* (the :bottom-circle-2 radius) (vec3 0 1 0)))
	   end (v+ (the :top-circle-2 center) (v* (the :top-circle-2 radius) (vec3 0 1 0)))
	   visible? (eq (the display) :wireframe))
   (line-3-2 :type 'line
	   start (v+ (the :bottom-circle-2 center) (v* (the :bottom-circle-2 radius) (vec3 -1 0 0)))
	   end (v+ (the :top-circle-2 center) (v* (the :top-circle-2 radius) (vec3 -1 0 0)))
	   visible? (eq (the display) :wireframe))
   (line-4-2 :type 'line
	   start (v+ (the :bottom-circle-2 center) (v* (the :bottom-circle-2 radius) (vec3 0 -1 0)))
	   end (v+ (the :top-circle-2 center) (v* (the :top-circle-2 radius) (vec3 0 -1 0)))
	   visible? (eq (the display) :wireframe))
   (s-1 :type 'solid-primitive-mixin
	body (the oc-solid)
	visible? nil)
   (s-2 :type 'solid-primitive-mixin
	body (the oc-solid-2)
	visible? nil)
   (:solid :type 'subtraction
	   objects (list (the s-1))
	   tools (list (the s-2))
	   visible? (eq (the display) :shaded))))


(defobject prism-mixin (block-mixin))

(defobject prism_a (prism-mixin)
  :inputs
  ((width :visual 1.0d0)
   (length :visual 1.0d0)
   (height :visual 1.0d0)
   (display :visual :wireframe))

  :attributes
  ((centroid (vec3 (half (the width)) (half (the length)) (half (the height)))))

  :components
  ((:edge-left-rear :type 'line
		    start (vec3 0 0 0)
		    end (v+ (the-part start) (vec3 0 0 (the height)))
		    visible? (eq (the display) :wireframe))
   (:edge-right-rear :type 'line
		     start (v+ (vec3 0 0 0) (vec3 (the width) 0 0))
		     end (v+ (the-part start) (vec3 0 0 (the height)))
		     visible? (eq (the display) :wireframe))
   (:edge-left-front :type 'line
		     start (v+ (vec3 0 0 0) (vec3 0 (the length) 0))
		     end (v+ (the-part start) (vec3 0 0 (the height)))
		     visible? (eq (the display) :wireframe))
   (:edge-right-front :type 'line
		      start (v+ (vec3 0 0 0) (vec3 (the width) (the length) 0))
		      end (v+ (the-part start) (vec3 0 0 (the height)))
		      visible? (eq (the display) :wireframe))
   (:edge-left-top :type 'line
		   start (the :edge-left-rear end)
		   end (the :edge-left-front end)
		   visible? (eq (the display) :wireframe))
   (:edge-right-top :type 'line
		    start (the :edge-right-rear end)
		    end (the :edge-right-front end)
		    visible? (eq (the display) :wireframe))
   (:edge-rear-top :type 'line
		   start (the :edge-left-rear end)
		   end (the :edge-right-rear end)
		   visible? (eq (the display) :wireframe))
   (:edge-front-top :type 'line
		    start (the :edge-left-front end)
		    end (the :edge-right-front end)
		    visible? (eq (the display) :wireframe))
   (:edge-left-bottom :type 'line
		      start (the :edge-left-rear start)
		      end (the :edge-left-front start)
		      visible? (eq (the display) :wireframe))
   (:edge-right-bottom :type 'line
		       start (the :edge-right-rear start)
		       end (the :edge-right-front start)
		       visible? (eq (the display) :wireframe))
   (:edge-rear-bottom :type 'line
		      start (the :edge-left-rear start)
		      end (the :edge-right-rear start)
		      visible? (eq (the display) :wireframe))
   (:edge-front-bottom :type 'line
		       start (the :edge-left-front start)
		       end (the :edge-right-front start)
		       visible? (eq (the display) :wireframe))))
		      
  



  
		   



  
(defmethod edges-connect? ((edge1 plane-curve-mixin) (edge2 plane-curve-mixin))
  (let ((edge1-start (send edge1 start))
	(edge1-end   (send edge1 end))
	(edge2-start (send edge2 start))
	(edge2-end   (send edge2 end)))
  (or (and (within-tolerance (vx edge1-start) (vx edge2-start))
	   (within-tolerance (vy edge1-start) (vy edge2-start)))
      (and (within-tolerance (vx edge1-start) (vx edge2-end))
	   (within-tolerance (vy edge1-start) (vy edge2-end)))
      (and (within-tolerance (vx edge1-end)   (vx edge2-start))
	   (within-tolerance (vy edge1-end)   (vy edge2-start)))
      (and (within-tolerance (vx edge1-end)   (vx edge2-end))
	   (within-tolerance (vy edge1-end)   (vy edge2-end))))))

(defmethod edges-connect? ((edge1 circle-mixin) edge2)
  nil)

(defmethod edges-connect? (edge1 (edge2 circle-mixin))
  nil)

(defmethod edges-connect? ((edge1 circle-mixin) (edge2 circle-mixin))
  nil)
    
(defun sort-edges-into-wire-chains (edges)
  (if (null (cdr edges))
	
      edges
	
      (let ((chains (mapcar #'(lambda (e) (list :head e)) edges))
	    (mutated?))
	  
	(flet ((mutate-chains ()
		 (let* ((two chains)
			(chain-a)
			(chain-b)
			(edge-a)
			(edge-b)
			(edge-c))

		   (tagbody
     
		    :begin

		      (unless (cdr two)
			(return-from mutate-chains))
			
		      (setq chain-a (car chains))
		      (setq chain-b (cadr two))
			
		      (setq edge-a (cadr chain-a))
		      (setq edge-b (cadr chain-b))
			
		      (if (edges-connect? edge-b edge-a)
			  (loop for edge in (cdr chain-b)
				do (push edge (cdr chain-a))
				finally (setf (cdr two) (cddr two)
					      mutated? t))
			  (progn
			    (setq edge-c (car (last chain-a)))
			      
			    (unless (eq edge-c edge-a)
			      (when (edges-connect? edge-b edge-c)
				(nconc chain-a (cdr chain-b))
				(setf (cdr two) (cddr two)
				      mutated? t)))))
			
		      (setf two (cdr two))
			
		      (go :begin)))))
	  (tagbody

	   :again

	     (setq mutated? nil)
	       
	     (dotimes (n (length chains))
	       (setf chains (append (cdr chains) (list (car chains))))
	       (mutate-chains))
	       
	     (when mutated?
	       (go :again)))
	    
	  (mapcar #'cdr chains)))))

(defun ray-intersects-line-segment? (origin direction point-a point-b)
  (let* ((p2 (v+ origin direction))
	 (x2 (vx p2))
	 (y2 (vy p2))
	 (x1 (vx origin))
	 (y1 (vy origin))
	 (x3 (vx point-a))
	 (y3 (vy point-a))
	 (x4 (vx point-b))
	 (y4 (vy point-b))
	 (x3-x4 (- x3 x4))
	 (y3-y4 (- y3 y4))
	 (denom (- (* (- x1 x2) y3-y4) (* (- y1 y2) x3-x4))))
    (if (zerop denom)
	nil
	(let ((s (/ (- (* (- x1 x3) y3-y4) (* (- y1 y3) x3-x4))
		    denom))
	      (u (/ (- (* (- x1 x3) (- y1 y2)) (* (- y1 y3) (- x1 x2)))
		    denom)))
	  (when (and (<= 0 s 1) (<= 0 u 1))
	    s)))))

(defun quadratic-formula (a b c)
  (let ((discriminant (sqrt (- (expt b 2) (* 4 a c))))
	(2a (* 2 a))
	(-b (- b)))
    (list (/ (+ -b discriminant) 2a)
	  (/ (- -b discriminant) 2a))))

(defun isolate-roots-vas-cf (f)
  "procedure to return isolated root intervals based on Vincent's method"
  ;; Improving the Performance of the Continued Fractions
  ;; Method Using New Bounds of Positive Roots
  ;; A. G. Akritas, A. W. Strzebonski, P. S. Vigklas
  ;; https://www.semanticscholar.org/reader/87211188ec3548cd115e82442daf2513d5c5aa9a p.267-268

  (let* ((p (map 'vector #'rationalize f))
	 #+NIL(denoms (map 'list #'denominator p))
	 #+NIL(gcd (apply #'gcd denoms)))

    #+NIL(setq p (map 'vector #'(lambda (rat) (* rat gcd)) p))

    (flet ((evaluate-polynomial (p x)
	     (loop for i from 0 below (length p)
		   sum (* (aref p i) (expt x i))))

	   (intrv (a b c d)
	     (let ((min)
		   (max))
	       ;; divide by zero,
	       ;; this is a problem
	       (when (zerop c)
		 
		 (setq min (/ (+ a b) d))
		 (setq max most-positive-fixnum))
	       (when (zerop d)
		 (setq min (/ (+ a b) c))
		 (setq max -11))

	       

	       (if min
		   (list min max)
		   (list (min (/ a c) (/ b d)) (max (/ a c) (/ b d))))))

	   (cauchy-bound (p)
	     (let ((temp (map 'list #'(lambda (c)
					(abs (/ c (aref p (1- (length p))))))
			      p)))
	       (1+ (apply #'max temp)))))
  
      (let ((root-list)
	    (interval-stack)
	    alpha
	    a b c d s
	    a1 b1 c1 d1 r p1 s1 
	    a2 b2 c2 d2 p2 s2)

	(tagbody
	 step-1

	   (setq root-list ())

	   (setq s (descartes-rule-of-signs p))

	   (when (zerop s)
	     (return-from isolate-roots-vas-cf nil))
	
	   (when (= 1 s)
	     (return-from isolate-roots-vas-cf (list (list 0 (cauchy-bound f)))))

	   (push (list 1 0 0 1 p s) interval-stack)

	 step-2
	   (unless interval-stack
	     (go exit))
	   
	   (multiple-value-setq (a b c d p s) (values-list (pop interval-stack)))

	 step-3
	   (setq alpha (/ (cauchy-bound p)))

	 step-4

	   (when (> alpha 16)
	     (setq p (transform-polynomial p 0 alpha))
	     (setq a (* alpha a))
	     (setq c (* alpha c))
	     (setq alpha 1))

	 step-5

	   (when (>= alpha 1)
	     (setq p (transform-polynomial p alpha (1+ alpha)))
	     (setq b (+ (* alpha a) b))
	     (setq d (+ (* alpha c) d)))

	   (when (zerop (evaluate-polynomial p 0))
	     (let ((root (/ b d)))
	       (push (list root root) root-list)
	       (setq p (subseq p 1))))

	   (setq s (descartes-rule-of-signs p))
	   
	   (when (zerop s)
	     (go step-2))
	   
	   (when (= 1 s)
	     (push (intrv a b c d) root-list)
	     (go step-2))

	 step-6

	   (setq p1 (transform-polynomial p 1 2))

	   (setq a1 a)
	   (setq b1 (+ a b))
	   (setq c1 c) 
	   (setq d1 (+ c d))
	   (setq r 0)

	   (when (zerop (evaluate-polynomial p1 0))
	     (let ((root (/ b1 d1)))
	       (push (list root root) root-list)
	       (setq p1 (subseq p1 1))
	       (setq r 1)))

	   (setq s1 (descartes-rule-of-signs p1))

	   (setq s2 (- s s1 r))  ;; (- s s1 r) ;;typo?

	   (setq a2 b)
	   (setq b2 (+ a b))
	   (setq c2 d)
	   (setq d2 (+ c d))

	 step-7

	   (when (> s2 1)
	     (setq p2 (transform-polynomial (reverse p) 1 2))
	     
	     (when (zerop (evaluate-polynomial p2 0))
	       (setq p2 (subseq p2 1)))
	     
	     (setq s2 (descartes-rule-of-signs p2))
	   
	     ;;step-8
	     (when (< s1 s2)
	       (let ((a-temp a1)
		     (b-temp b1)
		     (c-temp c1)
		     (d-temp d1)
		     (p-temp p1)
		     (s-temp s1))
		 (setq a1 a2
		       b1 b2
		       c1 c2
		       d1 d2
		       p1 p2
		       s1 s2)
		 (setq a2 a-temp
		       b2 b-temp
		       c2 c-temp
		       d2 d-temp
		       p2 p-temp
		       s2 s-temp)))
	   
	     ;;step-9

	     (when (zerop s1)
	       (go step-2))
	   
	     (if (= 1 s1)
		 (progn 
		 (push (intrv a1 b1 c1 d1) root-list))
		 (push (list a1 b1 c1 d1 p1 s1) interval-stack))
	     
	     ;;step-10

	     (when (zerop s2)
	       (go step-2))

	     (if (= 1 s2)
		 (progn 
		 (push (intrv a2 b2 c2 d2) root-list))
		 (push (list a2 b2 c2 d2 p2 s2) interval-stack))

	     (go step-2))

	   (when (zerop s1)
	     (go step-2))
	   
	   (if (= 1 s1)
	       (progn 	     

	       (push (intrv a1 b1 c1 d1) root-list))
	       (push (list a1 b1 c1 d1 p1 s1) interval-stack))

	   (go step-2)
	   
	 exit)

	(nreverse root-list)
	#+NIL
	(mapcar #'(lambda (entry) ;; what makes you think you can do this?
		    (list (/ (car entry) gcd) (/ (cadr entry) gcd))) (nreverse root-list))))))

(defun converge-on-root2 (polynomial start end epsilon)
  (if (< (- end start) epsilon)
      (list (coerce (/ (+ start end) 2) 'double-float))
      (flet ((has-sign-change? (a b)
	       (let ((val-a (evaluate-polynomial polynomial a))
		     (val-b (evaluate-polynomial polynomial b)))
		 (not (= (signum val-a) (signum val-b))))))
	(let ((middle (/ (+ start end) 2)))
	  (if (has-sign-change? start middle)
	      (converge-on-root2 polynomial start middle epsilon)
	      (converge-on-root2 polynomial middle end epsilon))))))

(defun transform-polynomial (polynomial a b)
  ;; transforms variable of polynomial to x=(a + s*(b-a)) and returns polynomial of s in monomial basis
  (let* ((n+1 (length polynomial))
	 (new-poly (make-array n+1 :initial-element 0)))
    (if (= a 0)
	(loop for k from 0 below n+1
	      do (setf (aref new-poly k) (* (expt b k) (aref polynomial k))))

	(let ((intermediate ()))
	  (loop for i from 0 below n+1
		do (let ((new-terms (make-array n+1 :initial-element 0)))
		     (loop for k from 0 to i
			   do (let ((bc (binomial-coefficient i k))
				    (a-pow (expt a (- i k)))
				    (b-minus-a-pow (expt (- b a) k)))
				(setf (aref new-terms k) (* bc a-pow b-minus-a-pow (aref polynomial i)))))
		     (push new-terms intermediate))
		finally (mapcar #'(lambda (terms)
				    (loop for i from 0 below n+1
					  do (incf (aref new-poly i) (aref terms i))))
				intermediate))))
    new-poly))

(defun descartes-rule-of-signs (polynomial)
  (let ((n 0)
	(last-sign)
	(next-sign))
    
    (do ((i 0 (1+ i)))
	((= i (1- (length polynomial))))

      (unless (zerop (elt polynomial i))
	(setq last-sign (signum (elt polynomial i))))

      (unless (zerop (elt polynomial (1+ i)))
	(setq next-sign (signum (elt polynomial (1+ i))))

	(when last-sign
	  (when (/= last-sign next-sign)
	    (incf n)))))
    n))

(defun real-roots (polynomial &optional (epsilon +epsilon+))
  (cond ((= (length polynomial) 3)
	 (let ((roots
		 (quadratic-formula (aref polynomial 2)
				    (aref polynomial 1)
				    (aref polynomial 0))))
	   (remove-if #'null
		      (mapcar #'(lambda (number)
				  (if (complexp number)
				      (when (within-tolerance (imagpart number) 0.0d0 epsilon)
					(* (signum (realpart number)) (abs number)))
				      number))
			      roots))))
			       
	(t (let* ((intervals (isolate-roots-vas-cf polynomial)))
	     (sort (apply #'append
			  (mapcar #'(lambda (interval)
				      (converge-on-root2 polynomial (car interval) (cadr interval) epsilon))
				  intervals))
		   #'<)))))

(defun intersection-polynomial-circle/2d-ray (circle start end)
  (let* ((c (send circle center))
	 (x0 (vx start))
	 (y0 (vy start))
	 (x1 (vx end))
	 (y1 (vy end))
	 (xc (vx c))
	 (yc (vy c))
	 (radius (send circle radius))
	 (C0 (+ (- (expt radius 2))
		(expt x0 2) (- (* 2 x0 xc)) (expt xc 2)
		(expt y0 2) (- (* 2 y0 yc)) (expt yc 2)))
	 (C1 (+ (- (* 2 (expt x0 2))) (* 2 x0 x1) (* 2 x0 xc) (- (* 2 x1 xc))
		(- (* 2 (expt y0 2))) (* 2 y0 y1) (* 2 y0 yc) (- (* 2 y1 yc))))
	 (C2 (+ (expt x0 2) (- (* 2 x0 x1)) (expt x1 2)
		(expt y0 2) (- (* 2 y0 y1)) (expt y1 2))))
    (vector C0 C1 C2)))







(defobject myblock (block-mixin)
  :components
  ((:arc-1 :type 'arc
	   start (vec2 0 0)
	   interior (vec2 1 0)
	   end (vec2 1 1))
   (:line-1 :type '2d-line
	    start (the :arc-1 start)
	    end (the :arc-1 end))
   (:b-spline-1 :type 'b-spline-plane-curve
		control-points (list (vec2 1 0)
				     (vec2 1 2)
				     (vec2 -1 2)
				     (vec2 -1 0)
				     (vec2 -1 -2)
				     (vec2 1 -2)
				     (vec2 1 0))
		weights (list 1 1/3 1/3 1 1/3 1/3 1)
		knot-vector (list 0 0 0 0 1/2 1/2 1/2 1 1 1 1))
   (:line-2 :type 'b-spline-3d-line
	    start (vec3 0 0 0)
	    end (vec3 0 0 1))
   (:circle-2 :type 'b-spline-3d-circle
	      center (vec3 2 0 0)
	      radius 1.0d0)
   (:tube-1 :type 'swept-surface
	    trajectory-curve (the :line-2)
	    section-curve (the :circle-2))))

(defobject test1 (node-mixin)
  :components
  ((myblock-1 :type 'myblock
	      rotation (qrotation (vec3 0 1 1) pi/2))))

(defobject dial-fixture (block-mixin)
  :inputs
  ((center :visual (vec2 0 0))
   (inner-radius :visual 13)
   (shelf-radius :visual 14.5)
   (outer-radius :visual 15.26)
   (thickness :visual 0.5)
   (shelf-thickness :visual 0.0625)
   (position-holes-radius :visual (+ (the inner-radius) 0.5))
   (slot-feature-radius :visual 15.005)
   (slot-feature-diameter :visual 0.53)
   (slot-feature-angular-spacing :visual (d2r 4.5))
   (number-of-features :visual 10))

  :attributes
  ((start-angle 0.0d0)
   (end-angle (* (the number-of-features) (the slot-feature-angular-spacing))))
  
  :components
  ((inner-arc :type 'arc
	      radius (the inner-radius)
	      start-angle (the end-angle)
	      end-angle 0
	      center (the center)
	      orientation :cw
	      visible? nil)
   (shelf-arc :type 'arc
	      radius (the shelf-radius)
	      start-angle (the end-angle)
	      end-angle 0
	      center (the center)
	      orientation :cw
	      visible? nil)
   (shelf-start-side :type '2d-line
		     start (the shelf-arc start)
		     end (the outer-arc end)
		     visible? nil)
   (shelf-end-side :type '2d-line
		   start (the outer-arc start)
		   end (the shelf-arc end)
		   visible? nil)
   (shelf-profile :type 'profile
		  outer-loop (list (the outer-arc)
				   (the shelf-end-side)
				   (the shelf-arc)
				   (the shelf-start-side))
		  visible? nil)
   (shelf :type 'extrusion
	  visible? nil
	  profile (the shelf-profile)
	  vector (v* (the shelf-thickness) (vec3 0 0 1)))
   (start-side :type '2d-line
	       start (the inner-arc start)
	       end (the outer-arc end)
	       visible? nil)
   (outer-arc :type 'arc
	      radius (the outer-radius)
	      start-angle 0
	      end-angle (the end-angle)
	      center (the center)
	      orientation :ccw
	      visible? nil)
   (outer-arc-more :type 'arc
		   radius (+ (* 10 +epsilon+) (the outer-radius))
		   start-angle 0
		   end-angle (the end-angle)
		   center (the center)
		   orientation :ccw
		   visible? nil)
   (end-side :type '2d-line
	     start (the outer-arc start)
	     end (the inner-arc end)
	     visible? nil)
   (basic-profile :type 'profile
		  outer-loop (list (the outer-arc)
				   (the end-side)
				   (the inner-arc)
				   (the start-side))
		  visible? nil)
   (basic-solid :type 'extrusion
		profile (the basic-profile)
		vector (vec3 0 0 (the thickness))
		visible? nil)
   (solid-int :type 'subtraction
	      objects (list (the basic-solid))
	      tools (mapcar #'(lambda (elt)
				(send elt solid))
			    (the slot-features list-elements))
	      visible? nil)
   (solid :type 'subtraction
	  objects (list (the solid-int))
	  tools (cons (the shelf)
		      (mapcar #'(lambda (elt)
				  (send elt solid))
			      (the bevel-features list-elements))))   
   (slot-features :type 'slot-feature
		  :aggregate (:size (the number-of-features))
		  angle (* (+ 0.5 (the-part index)) (the slot-feature-angular-spacing))
		  center (the center)
		  position-radius (the slot-feature-radius)
		  slot-diameter (the slot-feature-diameter)
		  outer-edge-radius (the outer-radius)
		  outer-edge (the outer-arc)
		  thickness (the thickness))
   
   (bevel-features :type 'bevel-feature
		   :aggregate (:size (the number-of-features))
		   edge1 (the (slot-features (the-part index)) slot-edge-rear)
		   angle2 (the (slot-features (the-part index)) angle)
		   edge2 (the outer-arc-more)
		   thickness (the thickness))))

(defobject bevel-feature (entity-mixin)
  :inputs
  ((edge1)
   (edge2)
   (angle pi/4)
   (angle2)
   (thickness))

  :attributes
  ((start (intersection-1 (the edge1) (the edge2) (the edge1 end) self))
   (orig-dir (the edge1 direction))
   (dir (rotate-vector (vec3 (vx (the orig-dir)) (vy (the orig-dir)) 0.0d0)
		       (qrotation (vec3 0 0 1) (* 3 pi/4)))))
   

  :components
  ((edge3 :type '2d-line
	  start (the start)
	  end (v+ (v* -0.125 (the edge1 direction)) (the start))
	  visible? nil)
   (edge4 :type '2d-line
	  end (the edge3 end)
	  span (v* (/ 10.125 (cos (the angle))) (vec2 (vx (the dir)) (vy (the dir))))
	  visible? nil)
   (edge5 :type '2d-line
	  start (the edge4 start)
	  end (the start)
	  visible? nil)
   (profile :type 'profile
	    outer-loop (list (the edge3)
			     (the edge4)
			     (the edge5))
	    visible? nil)
   (solid :type 'extrusion
	  visible? nil
	  profile (the profile)
	  vector (vec3 0 0 (the thickness)))))

(defun unsuppress-tree (node)
  (when (typep node 'node-mixin)
    (unsuppress-tree-1 node)
    (adhoc-scene-graph::rm-redraw-node-recursively node)))

(defun unsuppress-tree-1 (node)
  (when (typep node 'node-mixin)
    (setf (slot-value node 'adhoc::suppressed?) nil)
    (mapcar #'unsuppress-tree-1 (send node children))))

(defobject slot-feature (entity-mixin)
  :inputs
  ((angle)
   (center)
   (position-radius)
   (slot-diameter)
   (outer-edge-radius)
   (outer-edge)
   (thickness))

  :attributes
  ((slot-unit-vector (vec2 (cos (the angle)) (sin (the angle))))
   (slot-center (v* (the position-radius) (the slot-unit-vector))))

  :components
  ((centerline :type '2d-line
	       start (the center)
	       end (v* 1.5 (v* (the outer-edge-radius) (the slot-unit-vector)))
	       visible? nil)
   (slot-edge-front :type '2d-line-offset
		    curve (the centerline)
		    distance (- (/ (the slot-diameter) 2) +epsilon+)
		    visible? nil)
   (slot-edge-rear :type '2d-line-offset
		   curve (the centerline)
		   distance (+ +epsilon+ (- (/ (the slot-diameter) 2)))
		   visible? nil)
   (circle :type 'circle
	   center (the slot-center)
	   radius (/ (the slot-diameter) 2)
	   visible? nil))

  :attributes
  ((arc-start-point (intersection-1 (the circle) (the slot-edge-front) (the slot-edge-front end) self))
   (arc-end-point (intersection-1 (the circle) (the slot-edge-rear) (the slot-edge-rear end) self))
   (slot-front-start-point (intersection-1 (the slot-edge-front) (the outer-edge) (the slot-edge-front end) self))
   (slot-rear-start-point (intersection-1 (the slot-edge-rear) (the outer-edge) (the slot-edge-rear end) self)))

  :components
  ((arc-1 :type 'arc
	  center (the slot-center)
	  ;;radius (/ (the slot-diameter) 2)
	  start (the arc-start-point)
	  end (the arc-end-point)
	  orientation :ccw
	  visible? nil)
   (line-1 :type '2d-line
	   start (the arc-end-point)
	   end (the slot-rear-start-point)
	   visible? nil)
   (arc-2 :type 'arc
	  center (the center)
	  ;;radius (the outer-edge-radius)
	  start (the slot-front-start-point)
	  end (the slot-rear-start-point)
	  orientation :cw
	  visible? nil)
   (line-2 :type '2d-line
	   start (the slot-front-start-point)
	   end (the arc-start-point)
	   visible? nil)
   (profile :type 'profile
	    outer-loop (list (the arc-1)
			     (the line-1)
			     (the arc-2)
			     (the line-2))
	    visible? nil)
   (solid :type 'extrusion
	  visible? nil
	  profile (the profile)
	  vector (vec3 0 0 (the thickness)))))
		    
(defvar df)   
  
		   
   
   
(defun lowest-common-ancestor (object1 object2)
  (let ((root1 (send object1 root)))
    (when (eq root1 (send object2 root))
      (let ((rp1 (send object1 root-path))
	    (rp2 (send object2 root-path)))
	(loop for m1 in (rest rp1)
	      for m2 in (rest rp2)
	      with o1 = root1
	      until (not (eq m1 m2))
	      do (setq o1 (send o1 (eval m1)))
	      finally (return o1))))))

;;(transform-coordinate-from-to (the object1 center) object1 (lowest-common-ancestor object1 object2))

(defstruct axis1-placement
  (origin)
  (axis))

(defstruct axis2-placement
  (origin)
  (ref-dir)
  (axis))

(defun circle-ax2 (circle)
  (make-axis2-placement :origin (send circle center)
			:ref-dir (vec2 1 0)
			:axis (vec2 0 1)))

(defun line-ax1 (line)
  (make-axis1-placement :origin (send line start)
			:axis (send line direction)))


(defun transform-ax2! (ax2 from to)
  (setf (axis2-placement-origin ax2) (transform-coordinate-from-to (axis2-placement-origin ax2) from to))
  (setf (axis2-placement-ref-dir ax2) (transform-coordinate-from-to (axis2-placement-ref-dir ax2) from to))
  (setf (axis2-placement-axis ax2) (transform-coordinate-from-to (axis2-placement-axis ax2) from to))
  ax2)

(defun transform-ax1! (ax1 from to)
  (setf (axis1-placement-origin ax1) (transform-coordinate-from-to (axis1-placement-origin ax1) from to))
  (setf (axis1-placement-axis ax1) (transform-coordinate-from-to (axis1-placement-axis ax1) from to))
  ax1)

(defun coplanar-points? (pt1 pt2 pt3 pt4)
  (let* ((pt2-pt1 (v- pt2 pt1))
	 (pt3-pt1 (v- pt3 pt1))
	 (normal123 (vunit (vc pt2-pt1 pt3-pt1)))
	 (pt4-pt1 (v- pt4 pt1))
	 (normal124 (vunit (vc pt2-pt1 pt4-pt1))))
    (or (v= normal123 normal124)
	(v= (v- normal123) normal124))))

(defun ax2-points (ax2)
  (values (axis2-placement-origin ax2)
	  (v+ (axis2-placement-origin ax2) (axis2-placement-ref-dir ax2))
	  (v+ (axis2-placement-origin ax2) (axis2-placement-axis ax2))))

(defun ax1-points (ax1)
  (values (axis1-placement-origin ax1)
	  (v+ (axis1-placement-origin ax1) (axis1-placement-axis ax1))))

(defmethod coplanar? ((arc arc-mixin) (line line-mixin))
  (coplanar-arc-line? arc line))

(defmethod coplanar? ((line line-mixin) (arc arc-mixin))
  (coplanar-arc-line? arc line))

(defun coplanar-arc-line? (arc line)
  (let ((ax2 (circle-ax2 arc))
	(ax1 (line-ax1 line)))
    (let ((lca (lowest-common-ancestor arc line)))
      (transform-ax2! ax2 arc lca)
      (transform-ax1! ax1 line lca)
      (multiple-value-bind (pt1 pt2 pt3) (ax2-points ax2)
	(multiple-value-bind (pt4 pt5) (ax1-points ax1)
	  (let ((coplanar?
		  (and (coplanar-points? pt1 pt2 pt3 pt4)
		       (coplanar-points? pt1 pt2 pt3 pt5))))
	    (if coplanar?
		(values t (plane-rotation-matrix2 pt1 pt2 pt3) lca)
		(values nil nil lca))))))))

(defmethod coplanar? ((line-1 line-mixin) (line-2 line-mixin))
  (let ((ax1-1 (line-ax1 line-1))
	(ax1-2 (line-ax1 line-2)))
    (let ((lca (lowest-common-ancestor line-1 line-2)))
      (transform-ax1! ax1-1 line-1 lca)
      (transform-ax1! ax1-2 line-2 lca)
      (multiple-value-bind (pt1 pt2) (ax1-points ax1-1)
	(multiple-value-bind (pt3 pt4) (ax1-points ax1-2)
	  (if (coplanar-points? pt1 pt2 pt3 pt4)
	      (values t (plane-rotation-matrix2 pt1 pt2 pt3) lca)
	      (values nil nil lca)))))))

(defun plane-rotation-matrix (pt1 pt2 pt3)
  (let* ((normal (vunit (vc (v- pt2 pt1) (v- pt3 pt1))))
	 (theta (let ((th (acos (vz normal))))
		  (* (signum (realpart th)) (abs th))))
	 (rotation (vc normal (vec3 0 0 1)))
	 (skew-symmetric (mat4 0 (- (vz rotation)) (vy rotation) 0
			       (vz rotation) 0 (- (vx rotation)) 0
			       (- (vy rotation)) (vx rotation) 0 0
			       0 0 0 0))
	 (m1 (meye 4))
	 (m2 (scalar-times-matrix (sin theta) skew-symmetric))
	 (m3 (scalar-times-matrix (- 1 (cos theta)) (m* skew-symmetric skew-symmetric))))
    
    (binary-m+ m1 (binary-m+ m2 m3))))

(defun plane-rotation-matrix2 (pt1 pt2 pt3)
  (let* ((x-axis (vunit (v- pt2 pt1)))
	 (y-axis (vunit (v- pt3 pt1)))
	 (z-axis (vc x-axis y-axis)))
    (minv (mat4 (vx x-axis) (vx y-axis) (vx z-axis) 0
		(vy x-axis) (vy y-axis) (vy z-axis) 0
		(vz x-axis) (vz y-axis) (vz z-axis) 0
		0 0 0 1))))
	
  


		  

(defstruct intersection
  (coordinates)
  (coordinate-system))

(defmethod intersections-1 ((circle circle-mixin) (line line-mixin))
  (intersection-circle/line circle line))

(defmethod intersections-1 ((line line-mixin) (circle circle-mixin))
  (intersection-circle/line circle line))

(defmethod intersections-1 ((arc arc-mixin) (line line-mixin))
  (intersection-arc/line arc line))

(defmethod intersections-1 ((line line-mixin) (arc arc-mixin))
  (intersection-arc/line arc line))

(defun intersection-circle/line (circle line)
  (let ((lca (lowest-common-ancestor circle line))
	(ax1 (line-ax1 line))
	(ax2 (circle-ax2 circle)))
    (transform-ax1! ax1 line lca)
    (transform-ax2! ax2 circle lca)
    (multiple-value-bind (center pt2 pt3) (ax2-points ax2)
      (multiple-value-bind (start end) (ax1-points ax1)
	(let ((coplanar?
		(and (coplanar-points? center pt2 pt3 start)
		     (coplanar-points? center pt2 pt3 end))))
	  (if coplanar?
	      (let ((matrix (plane-rotation-matrix2 center pt2 pt3)))
		(setq center (safe-euclid (m* (vec4 (vx center) (vy center) (vz center) 1.0d0) matrix)))
		(setq pt2    (safe-euclid (m* (vec4 (vx pt2)    (vy pt2)    (vx pt2)    1.0d0) matrix)))
		(setq start  (safe-euclid (m* (vec4 (vx start)  (vy start)  (vz start)  1.0d0) matrix)))
		(setq end    (safe-euclid (m* (vec4 (vx end)    (vy end)    (vz end)    1.0d0) matrix)))
		(let* ((radius (send circle radius) #+NIL(vlength (v- pt2 center)))
		       (z (vz center))
		       (xc (vx center))
		       (yc (vy center))
		       (x0 (vx start))
		       (y0 (vy start))
		       (x1 (vx end))
		       (y1 (vy end))
		       (C0 (+ (- (expt radius 2))
			      (expt x0 2) (- (* 2 x0 xc)) (expt xc 2)
			      (expt y0 2) (- (* 2 y0 yc)) (expt yc 2)))
		       (C1 (+ (- (* 2 (expt x0 2))) (* 2 x0 x1) (* 2 x0 xc) (- (* 2 x1 xc))
			      (- (* 2 (expt y0 2))) (* 2 y0 y1) (* 2 y0 yc) (- (* 2 y1 yc))))
		       (C2 (+ (expt x0 2) (- (* 2 x0 x1)) (expt x1 2)
			      (expt y0 2) (- (* 2 y0 y1)) (expt y1 2)))
		       (roots (real-roots (vector C0 C1 C2)))
		       (points (mapcar #'(lambda (root) (lerp start end root))
				       roots)))
		  ;;(remove-if-not #'(lambda (root) (<= 0.0d0 root 1.0d0)) roots)
		  (when points
		    (let ((inverse (minv matrix)))
		      (make-intersection
		       :coordinates (mapcar #'(lambda (point)
						(safe-euclid
						 (m* (vec4 (vx point) (vy point) z 1.0d0) inverse)))
					    points)
		       :coordinate-system lca)))))))))))

(defun intersection-arc/line (arc line)
  (let ((lca (lowest-common-ancestor arc line))
	(ax1 (line-ax1 line))
	(ax2 (circle-ax2 arc)))
    (transform-ax1! ax1 line lca)
    (transform-ax2! ax2 arc lca)
    (multiple-value-bind (center pt2 pt3) (ax2-points ax2)
      (multiple-value-bind (start end) (ax1-points ax1)
	(let ((coplanar?
		(and (coplanar-points? center pt2 pt3 start)
		     (coplanar-points? center pt2 pt3 end))))
	  (if coplanar?
	      (let ((matrix (plane-rotation-matrix2 center pt2 pt3)))
		(setq center (safe-euclid (m* (vec4 (vx center) (vy center) (vz center) 1.0d0) matrix)))
		(setq pt2    (safe-euclid (m* (vec4 (vx pt2)    (vy pt2)    (vx pt2)    1.0d0) matrix)))
		(setq start  (safe-euclid (m* (vec4 (vx start)  (vy start)  (vz start)  1.0d0) matrix)))
		(setq end    (safe-euclid (m* (vec4 (vx end)    (vy end)    (vz end)    1.0d0) matrix)))
		(let ((astart (transform-coordinate-from-to (send arc start) arc lca))
		      (aend   (transform-coordinate-from-to (send arc end)   arc lca)))
		  (setq astart (safe-euclid (m* (vec4 (vx astart) (vy astart) (vz astart) 1.0d0) matrix)))
		  (setq aend   (safe-euclid (m* (vec4 (vx aend)   (vy aend)   (vz aend)   1.0d0) matrix)))
		  (let* ((radius (send arc radius)#+NIL(vlength (v- pt2 center)))
			 (z (vz center))
			 (xc (vx center))
			 (yc (vy center))
			 (x0 (vx start))
			 (y0 (vy start))
			 (x1 (vx end))
			 (y1 (vy end))
			 (C0 (+ (- (expt radius 2))
				(expt x0 2) (- (* 2 x0 xc)) (expt xc 2)
				(expt y0 2) (- (* 2 y0 yc)) (expt yc 2)))
			 (C1 (+ (- (* 2 (expt x0 2))) (* 2 x0 x1) (* 2 x0 xc) (- (* 2 x1 xc))
				(- (* 2 (expt y0 2))) (* 2 y0 y1) (* 2 y0 yc) (- (* 2 y1 yc))))
			 (C2 (+ (expt x0 2) (- (* 2 x0 x1)) (expt x1 2)
				(expt y0 2) (- (* 2 y0 y1)) (expt y1 2)))
			 (roots (real-roots (vector C0 C1 C2)))
			 (points (mapcar #'(lambda (root) (lerp start end root))
					 roots))
			 (orientation (send arc orientation))
			 #+NIL(center-to-start (v/ (v- astart center) radius))
			 #+NIL(center-to-end   (v/ (v- aend   center) radius))
			 #+NIL(start-angle (atan (vy center-to-start) (vx center-to-start)))
			 #+NIL(end-angle   (atan (vy center-to-end)   (vx center-to-end))))
		    ;; wonder if I need all this rigamorole for start-angle and end-angle
		    ;; isn't it invariant under transformation?
		    (let ((end-angle (send arc end-angle))
			  (start-angle (send arc start-angle)))
		    (setq points
			  (remove-if #'null
				     (mapcar #'(lambda (point)
						 (let* ((y (- (vy point) yc))
							(x (- (vx point) xc))
							(dir (vunit (vec2 x y)))
							(param (atan (vy dir) (vx dir))))
						 (if (eq orientation :ccw)
						     (when (>= end-angle param start-angle)
						       point)
						     (when (<= end-angle param start-angle)
						       point))))
					     points))))
		    (when points
		      (let ((inverse (minv matrix)))
			(make-intersection
			 :coordinates (mapcar #'(lambda (point)
						  (safe-euclid
						   (m* (vec4 (vx point) (vy point) z 1.0d0) inverse)))
					      points)
			 :coordinate-system lca))))))))))))

(defmethod clim:handle-event :after ((self node-mixin) (event clim:pointer-enter-event))
  (when (adhoc::component-definition self)
    (climish::message "~S ~A" (sb-mop::slot-definition-name (adhoc::component-definition self)) self)))

(defun intersection-line/line (line-a line-b)
  (let ((lca (lowest-common-ancestor line-a line-b))
	(ax1-a (line-ax1 line-a))
	(ax1-b (line-ax1 line-b)))
    (transform-ax1! ax1-a line-a lca)
    (transform-ax1! ax1-b line-b lca)
    (multiple-value-bind (start-a end-a) (ax1-points ax1-a)
      (multiple-value-bind (start-b end-b) (ax1-points ax1-b)
	(when (coplanar-points? start-a end-a start-b end-b)
	  (let ((matrix (plane-rotation-matrix2 start-a end-a start-b)))
	    (setq start-a (safe-euclid (m* (vec4 (vx start-a) (vy start-a) (vz start-a) 1.0d0) matrix)))
	    (setq end-a   (safe-euclid (m* (vec4 (vx end-a)   (vy end-a)   (vz end-a)   1.0d0) matrix)))
	    (setq start-b (safe-euclid (m* (vec4 (vx start-b) (vy start-b) (vz start-b) 1.0d0) matrix)))
	    (setq end-b   (safe-euclid (m* (vec4 (vx end-b)   (vy end-b)   (vz end-b)   1.0d0) matrix)))
	    (let ((z (vz start-a)))
		  
	      (setq start-a (vec2 (vx start-a) (vy start-a)))
	      (setq end-a   (vec2 (vx end-a)   (vy end-a)))
	      (setq start-b (vec2 (vx start-b) (vy start-b)))
	      (setq end-b   (vec2 (vx end-b)   (vy end-b)))

	      (let* ((span-a (v- start-a end-a))
		     (intersection (ray-intersects-line-segment? start-a span-a start-b end-b)))
		;; doesn't check for collinear lines.
		(when intersection
		  (let ((coordinate (v+ start-a (v* intersection span-a))))
		    (make-intersection
		     :coordinates (list (safe-euclid
					 (m* (vec4 (vx coordinate) (vy coordinate) z 1.0d0)
					     (minv matrix))))
		     :coordinate-system lca)))))))))))

(defmethod intersections-1 ((line-a line-mixin) (line-b line-mixin))
  (intersection-line/line line-a line-b))



(defun intersection-1 (entity-a entity-b pick coordinate-system)
  (let ((int (intersections-1 entity-a entity-b)))
    (when int
      (let ((coords (intersection-coordinates int)))
	(if (cdr coords)
	    ;; pick the coordinate closest to pick
	    (if (eq pick t)
		(let ((coord (transform-coordinate-from-to (car coords)
							   (intersection-coordinate-system int)
							   coordinate-system)))
		  (if (zerop (vz coord))
		      (vec2 (vx coord) (vy coord))
		      coord))
		(progn
		  (when (typep pick 'vec2)
		    (setq pick (vec3 (vx pick) (vy pick) 0.0d0)))
		  (loop with min = most-positive-double-float
			for coord in coords
			with len
			with result
			do (setq coord (transform-coordinate-from-to
					coord (intersection-coordinate-system int)
					coordinate-system))
			when (< (setq len (vlength (v- pick coord))) min)
			  do (setq result coord)
			     (setq min len)
			finally (return (if (zerop (vz result))
					    (vec2 (vx result) (vy result))
					    result)))))
	    (let ((coord (transform-coordinate-from-to (car coords)
						       (intersection-coordinate-system int)
						       coordinate-system)))
	      (if (zerop (vz coord))
		  (vec2 (vx coord) (vy coord))
		  coord)))))))
								   
		    

(defun binary-m+ (m1 m2)
  (3dm.d::with-fast-matref (a m1 4)
    (3dm.d::with-fast-matref (b m2 4)
      (mat4 (+ (a 0 0) (b 0 0)) (+ (a 0 1) (b 0 1)) (+ (a 0 2) (b 0 2)) (+ (a 0 3) (b 0 3))
	    (+ (a 1 0) (b 1 0)) (+ (a 1 1) (b 1 1)) (+ (a 1 2) (b 1 2)) (+ (a 1 3) (b 1 3))
	    (+ (a 2 0) (b 2 0)) (+ (a 2 1) (b 2 1)) (+ (a 2 2) (b 2 2)) (+ (a 2 3) (b 2 3))
	    (+ (a 3 0) (b 3 0)) (+ (a 3 1) (b 3 1)) (+ (a 3 2) (b 3 2)) (+ (a 3 3) (b 3 3))))))

(defun scalar-times-matrix (scalar matrix)
  (3dm.d::with-fast-matref (mref matrix 4)
    (mat4 (* scalar (mref 0 0)) (* scalar (mref 0 1)) (* scalar (mref 0 2)) (* scalar (mref 0 3))
	  (* scalar (mref 1 0)) (* scalar (mref 1 1)) (* scalar (mref 1 2)) (* scalar (mref 1 3))
	  (* scalar (mref 2 0)) (* scalar (mref 2 1)) (* scalar (mref 2 2)) (* scalar (mref 2 3))
	  (* scalar (mref 3 0)) (* scalar (mref 3 1)) (* scalar (mref 3 2)) (* scalar (mref 3 3)))))

(defobject 3d-circlish-mixin (space-curve-mixin))

(defobject 3d-arc-mixin (3d-circlish-mixin))

(defobject 3d-arc-mixin1 (3d-arc-mixin)
  :inputs
  ((start)
   (interior)
   (end))
  
  :attributes
  ((plane-x-axis (vunit (v- (the start) (the interior))))
   (%plane-y-ish-axis (vunit (v- (the end) (the interior))))
   (normal (vc (the plane-x-axis) (the %plane-y-ish-axis)))
   (plane-y-axis (vc (the plane-x-axis) (the normal)))
   (%plane-rotation-matrix
    (let ((x-axis (the plane-x-axis))
	  (y-axis (the plane-y-axis))
	  (z-axis (the normal)))
      (mat4 (vx x-axis) (vx y-axis) (vx z-axis) 0
	    (vy x-axis) (vy y-axis) (vy z-axis) 0
	    (vz x-axis) (vz y-axis) (vz z-axis) 0
	    0 0 0 1)))
   (%inv-plane-rotation-matrix (minv (the %plane-rotation-matrix)))
   (%start (let* ((p (the start)))
	     (safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%interior (let* ((p (the interior)))
		(safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%end (let* ((p (the end)))
	   (safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%center-radius-degenerate
    (multiple-value-list (%circle-from-3pts (the %start) (the %interior) (the %end))))
   (%z (vz (the %start)))
   (%center (let ((p (first (the %center-radius-degenerate))))
	      (vec3 (vx p) (vy p) (the %z))))
   (radius (second (the %center-radius-degenerate)))
   (degenerate? (third (the %center-radius-degenerate)))
   (center (let ((p (the %center)))
	     (safe-euclid (m* (the %plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%center-to-end (v- (the %end) (the %center)))
   (%center-to-start (v- (the %start) (the %center)))
   (start-angle (let* ((dir (the %center-to-start)))
		   (atan (vy dir) (vx dir))))
   (end-angle (let* ((dir (the %center-to-end)))
		 (atan (vy dir) (vx dir))))
   (delta-angle (let* ((start-angle (the start-angle))
		       (end-angle (the end-angle))
		       (a (- end-angle start-angle)))
		  (if (< a 0.0d0)
		      (+ a 2pi)
		      a)))
   (orientation :uncached (if (plusp (vz (vc (v- (the %interior) (the %start))
					     (v- (the %interior) (the %end)))))
				     :ccw
				     :cw))
   (domain :uncached (list (the start-angle) (the end-angle)))))

(defmethod evaluate1 ((self 3d-circlish-mixin) (parameter real) &key (derivs 0))
  (declare (ignorable derivs))
  (let* ((center (the %center))
	 (radius (the radius))
	 (z (the %z))
	 (matrix (the %plane-rotation-matrix))
	 (point (safe-euclid
		 (m* matrix (vec4 (+ (vx center) (* radius (cos parameter)))
				  (+ (vy center) (* radius (sin parameter)))
				  z
				  1.0d0)))))
    (cond ((= derivs 0) point)
	  ((= derivs 1)
	   (values point (safe-euclid
			  (m* matrix (vec4 (- (sin parameter))
					   (cos parameter)
					   0.0d0
					   1.0d0)))))
	  ((> derivs 1)
	   (values point
		   (safe-euclid
		    (m* matrix (vec4 (- (sin parameter))
				     (cos parameter)
				     0.0d0
				     1.0d0)))
		   (safe-euclid
		    (m* matrix (vec4 (- (cos parameter))
				     (- (sin parameter))
				     0.0d0
				     1.0d0))))))))

	  
  

(defobject 3d-arc_a (3d-arc-mixin1 entity-mixin))

(defmethod rm-draw-node ((self 3d-arc-mixin))
  (when (the visible?)
    (let* ((scene  (the krma:scene))
	   (center (the %center))
	   (z (the %z))
	   (radius (the radius))
	   (delta-theta (the delta-angle))
	   (sign (signum delta-theta))
	   (niter (floor (abs (/ (* delta-theta 180.0d0) pi))))
	   (matrix (the %plane-rotation-matrix))
	   (oid    (the object-id))
	   (verts (make-array niter))
	   (start-angle (the start-angle)))
      (loop for i from 0 below niter
	    with theta = (the end-angle)
	    with step = #.(/ pi 180.0d0)
	    do (let ((angle (- theta (* sign i step))))
		 (setf (aref verts i)
		       (safe-euclid
			(m* matrix (vec4 (+ (vx center) (* radius (cos angle)))
					 (+ (vy center) (* radius (sin angle)))
					 z
					 1.0d0))))))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff
	     (nconc
	      (let ((start (safe-euclid
			    (m* matrix (vec4 (+ (vx center) (* radius (cos start-angle)))
					     (+ (vy center) (* radius (sin start-angle)))
					     z
					     1.0d0)))))
		(list (vx start) (vy start) (vz start)))
	      (loop for coord across verts
		    with vertices = ()
		    do (setq vertices (cons (vx coord) (cons (vy coord) (cons (vz coord) vertices))))
		    finally (return vertices)))
	     oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defobject 3d-circle-mixin (3d-circlish-mixin)
  :inputs
  ((center (:validator (assure-double-vec3 value)))
   (normal (:validator (let ((n (assure-double-vec3 value)))
			 (vunit n))))
   (radius (:validator (assure-nonnegative-double-float value))))

  :attributes
  ((plane-x-axis (let ((un (the normal)))
		   (if (within-tolerance 1.0d0 (v. un (vec3 0 1 0)) 0.1d0)
		       (vunit (vc un (vec3 0 0 1)))
		       (vunit (vc (vec3 0 1 0) un)))))
   (plane-y-axis (vunit (vc (the normal) (the plane-x-axis))))
   (%plane-rotation-matrix
    (let ((x-axis (the plane-x-axis))
	  (y-axis (the plane-y-axis))
	  (z-axis (the normal)))
      (mat4 (vx x-axis) (vx y-axis) (vx z-axis) 0
	    (vy x-axis) (vy y-axis) (vy z-axis) 0
	    (vz x-axis) (vz y-axis) (vz z-axis) 0
	    0 0 0 1)))
   (%inv-plane-rotation-matrix (minv (the %plane-rotation-matrix)))
   (%center (let ((p (the center)))
	      (safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%z (vz (the %center)))
   (%start-and-end (let* ((p (vec3 (+ (vx (the center)) (the radius)) (vy (the center)) 0.0d0)))
	     (safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%interior (let* ((p (vec3 (- (vx (the center)) (the radius)) (vy (the center)) 0.0d0)))
		(safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))))

(defmethod to-opencascade ((self 3d-circle-mixin))
  (let* ((start-and-end (gp:pnt (vx (the %start-and-end)) (vy (the %start-and-end)) (vz (the %start-and-end))))
	 (interior (gp:pnt (vx (the %interior)) (vy (the %interior)) (vz (the %interior))))
	 (normal (gp:dir (vx (the normal)) (vy (the normal)) (vz (the normal))))
	 (center (gp:pnt (vx (the center)) (vy (the center)) (vz (the center))))
	 (ax2 (gp:ax2 :p center :n normal))
	 (circ (gp:circ :a2 ax2 :r (the radius))))
    (list (oc::value (make-instance 'oc::gc-make-arc-of-circle
				    :p1 start-and-end
				    :p2 interior
				    :circ circ
				    :sense t))
	  (oc::value (make-instance 'oc::gc-make-arc-of-circle
				    :p1 interior
				    :p2 start-and-end
				    :circ circ
				    :sense t)))))
  
  

(defobject 3d-circle_a (3d-circle-mixin entity-mixin))

(defmethod rm-draw-node ((self 3d-circle-mixin))
  (when (the visible?)
    (let* ((scene  (the krma:scene))
	   (center (the %center))
	   (z (the %z))
	   (radius (the radius))
	   (matrix (the %plane-rotation-matrix))
	   (oid    (the object-id))
	   (verts (make-array 360)))
      (loop for i from 0 below 360
	    with theta = 0.0d0
	    with step = #.(/ pi 180.0d0)
	    do (let ((angle (- theta (* i step))))
		 (setf (aref verts i)
		       (safe-euclid
			(m* matrix (vec4 (+ (vx center) (* radius (cos angle)))
					 (+ (vy center) (* radius (sin angle)))
					 z
					 1.0d0))))))
      (push (krma:scene-add-filled-3d-instanced-tube-primitive
	     scene (the group) nil nil 2.5f0 #xff
	     (nconc
	      (let ((start
		      (safe-euclid
		       (m* matrix (vec4 (+ (vx center) radius) (vy center) z 1.0d0)))))
		(list (vx start) (vy start) (vz start)))
	      (loop for coord across verts
		    with vertices = ()
		    do (setq vertices (cons (vx coord) (cons (vy coord) (cons (vz coord) vertices))))
		    finally (return vertices)))
	     oid)
	    (the primitive-handles))
      (setf (krma:object-from-id oid) self)))
  (values))

(defobject 3d-arc-mixin2 (3d-arc-mixin)
  
  :inputs
  ((center (:validator (assure-double-vec3 value)))
   (normal (:validator (let ((n (assure-double-vec3 value)))
			 (vunit n))))
   (radius (:validator (assure-nonnegative-double-float value)))
   (start-angle (:validator (assure-double-float value)))
   (end-angle (:validator (assure-double-float value)))
   (ref-direction (:validator (let ((n (assure-double-vec3 value)))
				(vunit n)))))
  
  :attributes
  ((plane-y-axis (vunit (vc (the normal) (the ref-direction))))
   (plane-x-axis (vunit (vc (the plane-y-axis) (the normal))))
   (%plane-rotation-matrix
    (let ((x-axis (the plane-x-axis))
	  (y-axis (the plane-y-axis))
	  (z-axis (the normal)))
      (mat4 (vx x-axis) (vx y-axis) (vx z-axis) 0
	    (vy x-axis) (vy y-axis) (vy z-axis) 0
	    (vz x-axis) (vz y-axis) (vz z-axis) 0
	    0 0 0 1)))
   (%inv-plane-rotation-matrix (minv (the %plane-rotation-matrix)))
   (%center (let ((p (the center)))
	      (safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%z (vz (the %center)))
   (%start (v+ (the %center)
	       (v* (the radius)
		   (vec3 (cos (the start-angle)) (sin (the start-angle)) 0.0d0))))
   (%end (v+ (the %center)
	     (v* (the radius)
		 (vec3 (cos (the end-angle)) (sin (the end-angle)) 0.0d0))))
   (start (let ((p (the %start)))
	    (safe-euclid (m* (the %plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (end (let ((p (the %end)))
	  (safe-euclid (m* (the %plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (delta-angle (let* ((start-angle (the start-angle))
		       (end-angle (the end-angle))
		       (a (- end-angle start-angle)))
		  (if (< a 0.0d0)
		      (+ a 2pi)
		      a)))
   (orientation :uncached :ccw)
   (degenerate? :uncached (<= (the radius) 0.0d0))
   (domain :uncached (list (the start-angle) (the end-angle)))))


  
(defobject 3d-arc_b (3d-arc-mixin2 entity-mixin))

(defobject 3d-arc-mixin3 (3d-arc-mixin)
  
  :inputs
  ((center (:validator (assure-double-vec3 value)))
   (start-point (:validator (assure-double-vec3 value)))
   (end-point (:validator (assure-double-vec3 value)))
   (orientation (:validator (assert (or (eq value :ccw) (eq value :cw))) value) :ccw))
  
  :attributes
  ((%%%center-to-start (v- (the start-point) (the center)))
   (plane-x-axis (vunit (the %%%center-to-start)))
   (%%%center-to-end (v- (the end-point) (the center)))
   (normal (vunit (vc (the %%%center-to-end) (the %%%center-to-start))))
   (plane-y-axis (vunit (vc (the normal) (the plane-x-axis))))
   (radius (min (vlength (the %%%center-to-start))
		(vlength (the %%%center-to-end))))
   (%%center-to-start (v* (the radius) (vunit (the %%%center-to-start))))
   (%%center-to-end (v* (the radius) (vunit (the %%%center-to-end))))
   (%plane-rotation-matrix
    (let ((x-axis (the plane-x-axis))
	  (y-axis (the plane-y-axis))
	  (z-axis (the normal)))
      (mat4 (vx x-axis) (vx y-axis) (vx z-axis) 0
	    (vy x-axis) (vy y-axis) (vy z-axis) 0
	    (vz x-axis) (vz y-axis) (vz z-axis) 0
	    0 0 0 1)))
   (%inv-plane-rotation-matrix (minv (the %plane-rotation-matrix)))
   (%center (let ((p (the center)))
	      (safe-euclid (m* (the %inv-plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (%z (vz (the %center)))
   (%center-to-start (let ((v (the %%center-to-start)))
		       (safe-euclid (m* (the %inv-plane-rotation-matrix)
					(vec4 (vx v) (vy v) (vz v) 1.0d0)))))
   (%center-to-end (let ((v (the %%center-to-end)))
		     (safe-euclid (m* (the %inv-plane-rotation-matrix)
				      (vec4 (vx v) (vy v) (vz v) 1.0d0)))))
   (%start (v+ (the %center) (the %center-to-start)))
   (%end (v+ (the %center) (the %center-to-end)))
   (start (let ((p (the %start)))
	    (safe-euclid (m* (the %plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (end (let ((p (the %end)))
	  (safe-euclid (m* (the %plane-rotation-matrix) (vec4 (vx p) (vy p) (vz p) 1.0d0)))))
   (start-angle :uncached 0.0d0)
   (end-angle (let ((v (the %center-to-end)))
		(atan (vy v) (vx v))))
   (delta-angle (let* ((a (the end-angle)))
		  (if (eq (the orientation) :ccw)
		      (if (< a 0.0d0)
			  (+ a 2pi)
			  a)
		      (if (< a 0)
			  a
			  (- a 2pi)))))
   (degenerate? :uncached (<= (the radius) 0.0d0))
   (domain :uncached (list (the start-angle) (the end-angle)))))

(defobject 3d-arc_c (3d-arc-mixin3 entity-mixin))

(defobject arc-test3 (block-mixin)
  :components
  ((arc :type '3d-arc_c
	center (vec3 1 1 1)
	start-point (vec3 1 0 0)
	end-point (vec3 0 1 0)
	orientation :cw)))
  

(defobject arc-test2 (block-mixin)
  :components
  ((arc :type '3d-arc_b
	center (vec3 1 0 0)
	normal (vec3 -1 -1 -1)
	radius 0.5
	ref-direction (vec3 -1 1 0)
	start-angle 0
	end-angle pi)))
	

(defobject circle-test (block-mixin)
  :components
  ((circle :type '3d-circle_a
	   center (vec3 1 1 1)
	   normal (vec3 0 1 0)
	   radius 1.0d0)
   (circle2 :type '3d-circle_a
	    center (vec3 0 1 0)
	    normal (vec3 1 1 1)
	    radius 0.5)))
  

(defobject arc-test (block-mixin)
  :components
  ((arc :type '3d-arc_a
	start (vec3 1 0 0)
	interior (vec3 0 0 1)
	end (vec3 0 1 0))))
