#!/usr/bin/env roseus

(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(load "models/room73b2-scene.l")
(nextage-init)

(defparameter *toromi-robot* *nextage*)
(defparameter *toromi-ri* *ri*)
(defparameter *stir-arm* :rarm)

;; Units are millimeters and milliseconds unless otherwise noted.
;; TODO: replace with measured cup center in the Nextage waist-local frame.
(defparameter *cup-pos* #f(435 -15 215))
(defparameter *cup-height* 100.0)
(defparameter *cup-depth* 95.0)
(defparameter *cup-top-z-offset* (/ *cup-height* 2.0))

(defparameter *spoon-tip-offset-from-rhand* #f(0 0 180))
(defparameter *stir-radius* 8.0)
(defparameter *stir-z-line-half-range* 3.0)
(defparameter *stir-depth-from-cup-top* 105.0)
(defparameter *stir-duration* 120000)
(defparameter *stir-step-time* 40)
(defparameter *stir-approach-height* 30.0)
(defparameter *stir-rpy* (list (deg2rad 90) 0 0))
(defparameter *scoop-tilted-rpy* (list (deg2rad 90) (deg2rad -35) 0))
(defparameter *stir-cycle-steps* 12)
(defparameter *stir-linear-cycle-steps* 13)
(defparameter *stir-repeat-count* 5)
(defparameter *stir-pause-time* 10000)
(defparameter *scoop-lift-height* 3.0)
(defparameter *pre-stir-right-offset* #f(0 -70 0))
(defparameter *pre-stir-height* 55.0)
(defparameter *pre-stir-time* 3000)
(defparameter *stir-approach-time* 3000)
(defparameter *stir-exit-time* 2000)

(defun robot-local-pos (pos)
  (send *toromi-robot* :transform-vector pos))

(defun cup-top-pos ()
  (v+ (robot-local-pos *cup-pos*) (float-vector 0 0 *cup-top-z-offset*)))

(defun cup-stir-center-pos ()
  (v+ (cup-top-pos) (float-vector 0 0 (- *stir-depth-from-cup-top*))))

(defun target-coords-from-tool-tip (tip-pos rpy tool-offset)
  (send (make-coords :pos tip-pos :rpy rpy)
        :translate
        (scale -1.0 tool-offset)))

(defun rhand-spoon-coords (tip-pos &optional (rpy *stir-rpy*))
  (target-coords-from-tool-tip
   tip-pos
   rpy
   *spoon-tip-offset-from-rhand*))

(defun solve-rarm-ik (target-coords)
  (let ((result (send *toromi-robot* *stir-arm*
                      :inverse-kinematics
                      target-coords
                      :revert-if-fail nil
                      :look-at-target nil
                      :rotation-axis :z
                      :debug-view nil)))
    (unless result
      (error "IK failed for ~A" *stir-arm*))
    result))

(defun current-av ()
  (send *toromi-robot* :angle-vector))

(defun execute-av-sequence (av-seq time-seq &key (wait t))
  (send *toromi-ri* :angle-vector-sequence av-seq time-seq)
  (when wait (send *toromi-ri* :wait-interpolation)))

(defun reset-pose (&key (execute t) (time 3000))
  (send *toromi-robot* :reset-pose)
  (send *toromi-robot* :head :angle-vector #f(0 30))
  (send *toromi-robot* :larm :move-end-pos #f(0 0 100))
  (send *toromi-robot* :rarm :move-end-pos #f(0 0 100))
  (when execute
    (send *toromi-ri* :angle-vector (send *toromi-robot* :angle-vector) time)
    (send *toromi-ri* :wait-interpolation))
  (send *toromi-robot* :angle-vector))

(defun finish-rarm-stir ()
  (let ((target (send (send *toromi-robot* *stir-arm* :end-coords)
                      :copy-worldcoords)))
    (send target :translate (float-vector 0 0 *pre-stir-height*) :world)
    (solve-rarm-ik target)
    (send *toromi-ri* :angle-vector (send *toromi-robot* :angle-vector) 3000)
    (send *toromi-ri* :wait-interpolation)

    (solve-rarm-ik (rhand-spoon-coords (pre-stir-tip-pos)))
    (send *toromi-ri* :angle-vector (send *toromi-robot* :angle-vector) 3000)
    (send *toromi-ri* :wait-interpolation)

    (reset-pose :execute t)))

(defun pre-stir-tip-pos ()
  (v+ (v+ (cup-top-pos) *pre-stir-right-offset*)
      (float-vector 0 0 *pre-stir-height*)))

(defun stir-tip-pos (theta &optional (z-offset 0.0))
  (let ((center (cup-stir-center-pos)))
    (v+ center
        (float-vector (* *stir-radius* (cos theta))
                      (* *stir-radius* (sin theta))
                      z-offset))))

(defun stir-tip-pos-from-offset (x y z &optional (z-offset 0.0))
  (v+ (cup-stir-center-pos)
      (float-vector x y (+ z z-offset))))

(defun line-motion-p (motion)
  (memq motion (list :x-line :y-line :z-line)))

(defun stir-motion-cycle-steps (motion)
  (if (line-motion-p motion)
      *stir-linear-cycle-steps*
    *stir-cycle-steps*))

(defun stir-motion-cycle-time (motion)
  (* *stir-cycle-steps* *stir-step-time*))

(defun stir-motion-step-times (motion)
  (let* ((cycle-steps (stir-motion-cycle-steps motion))
         (cycle-time (stir-motion-cycle-time motion))
         (base-time (floor (/ cycle-time (float cycle-steps))))
         (extra-count (mod cycle-time cycle-steps)))
    (let (time-seq)
      (dotimes (i cycle-steps)
        (push (+ base-time (if (< i extra-count) 1 0)) time-seq))
      (reverse time-seq))))

(defun stir-linear-offset (cycle-i &optional (half-range *stir-radius*))
  (let* ((phase (/ cycle-i (float (max 1 (- *stir-linear-cycle-steps* 1)))))
         (down-phase 0.25)
         (center-phase 0.5)
         (up-phase 0.75))
    (* half-range
       (cond
        ((< phase down-phase)
         (- (/ phase down-phase)))
        ((< phase center-phase)
         (+ -1.0 (/ (- phase down-phase)
                    (- center-phase down-phase))))
        ((< phase up-phase)
         (/ (- phase center-phase)
            (- up-phase center-phase)))
        (t
         (- 1.0 (/ (- phase up-phase)
                   (- 1.0 up-phase))))))))

(defun stir-motion-target (motion i &optional (z-offset 0.0))
  (let* ((cycle-steps (stir-motion-cycle-steps motion))
         (cycle-i (mod i cycle-steps))
         (theta (* 2 pi (/ cycle-i (float cycle-steps)))))
    (cond
     ((eq motion :circle)
      (list (stir-tip-pos theta z-offset) *stir-rpy*))
     ((eq motion :reverse-circle)
      (list (stir-tip-pos (- theta) z-offset) *stir-rpy*))
     ((eq motion :x-line)
      (list (stir-tip-pos-from-offset (stir-linear-offset cycle-i) 0 0 z-offset)
            *stir-rpy*))
     ((eq motion :y-line)
      (list (stir-tip-pos-from-offset 0 (stir-linear-offset cycle-i) 0 z-offset)
            *stir-rpy*))
     ((eq motion :z-line)
      (list (stir-tip-pos-from-offset
             0 0 (stir-linear-offset cycle-i *stir-z-line-half-range*) z-offset)
            *stir-rpy*))
     ((eq motion :scoop)
      (cond
       ((< cycle-i 5)
        (list (stir-tip-pos-from-offset
               0 0
               (- *scoop-lift-height*
                  (* 2 *scoop-lift-height* (/ cycle-i 4.0)))
               z-offset)
              *stir-rpy*))
       ((< cycle-i 9)
        (list (stir-tip-pos-from-offset
               0 0
               (+ (- *scoop-lift-height*)
                  (* 2 *scoop-lift-height* (/ (- cycle-i 4) 4.0)))
               z-offset)
              *stir-rpy*))
       ((< cycle-i 11)
        (list (stir-tip-pos-from-offset 0 0 *scoop-lift-height* z-offset)
              *scoop-tilted-rpy*))
       (t
        (list (stir-tip-pos-from-offset 0 0 *scoop-lift-height* z-offset)
              *stir-rpy*))))
     (t
      (error "Unknown stir motion: ~A" motion)))))

(defun plan-rarm-stir (&key (motion :circle) (mode :continuous))
  (let ((steps (max 1 (round (/ (float *stir-duration*) *stir-step-time*))))
        av-seq
        time-seq)
    (solve-rarm-ik (rhand-spoon-coords (pre-stir-tip-pos)))
    (push (current-av) av-seq)
    (push *pre-stir-time* time-seq)

    (let ((target (stir-motion-target motion 0 *stir-approach-height*)))
      (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
      (push (current-av) av-seq)
      (push *stir-approach-time* time-seq))

    (let ((step-times (stir-motion-step-times motion)))
      (cond
       ((eq mode :continuous)
        (dotimes (i steps)
          (let ((target (stir-motion-target motion i)))
            (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
            (push (current-av) av-seq)
            (push (nth (mod i (length step-times)) step-times) time-seq))))
       ((eq mode :once)
        (dotimes (i (stir-motion-cycle-steps motion))
          (let ((target (stir-motion-target motion i)))
            (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
            (push (current-av) av-seq)
            (push (nth i step-times) time-seq))))
       ((eq mode :five)
        (dotimes (j *stir-repeat-count*)
          (dotimes (i (stir-motion-cycle-steps motion))
            (let ((target (stir-motion-target motion i)))
              (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
              (push (current-av) av-seq)
              (push (nth i step-times) time-seq)))
          (when (< j (- *stir-repeat-count* 1))
            (push (current-av) av-seq)
            (push *stir-pause-time* time-seq))))
       (t
        (error "Unknown stir mode: ~A" mode))))

    #|
    (let ((target (stir-motion-target motion 0 *stir-approach-height*)))
      (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
      (push (current-av) av-seq)
      (push *stir-exit-time* time-seq))
    |#

    (list (reverse av-seq) (reverse time-seq))))

(defun plan-rarm-stir-circle ()
  (let ((steps (max 1 (round (/ (float *stir-duration*) *stir-step-time*))))
        av-seq
        time-seq)
    (solve-rarm-ik (rhand-spoon-coords (pre-stir-tip-pos)))
    (push (current-av) av-seq)
    (push *pre-stir-time* time-seq)

    (solve-rarm-ik (rhand-spoon-coords
                    (stir-tip-pos 0 *stir-approach-height*)))
    (push (current-av) av-seq)
    (push *stir-approach-time* time-seq)

    (dotimes (i steps)
      (let ((theta (* 2 pi (/ i 12.0))))
        (solve-rarm-ik (rhand-spoon-coords (stir-tip-pos theta)))
        (push (current-av) av-seq)
        (push *stir-step-time* time-seq)))

    #|
    (solve-rarm-ik (rhand-spoon-coords
                    (stir-tip-pos 0 *stir-approach-height*)))
    (push (current-av) av-seq)
    (push *stir-exit-time* time-seq)
    |#

    (list (reverse av-seq) (reverse time-seq))))

(defun rarm-stir-demo (&key (execute t) (motion :circle) (mode :continuous))
  "Move from the initial pose, keep the left arm fixed, and stir with the right arm."
  (reset-pose :execute nil)
  (let ((plan (if (and (eq motion :circle) (eq mode :continuous))
                  (plan-rarm-stir-circle)
                (plan-rarm-stir :motion motion :mode mode))))
    (when execute
      (reset-pose :execute t)
      (execute-av-sequence (car plan) (cadr plan))
      (send *toromi-robot* :angle-vector (car (last (car plan))))
      (finish-rarm-stir))
    plan))
