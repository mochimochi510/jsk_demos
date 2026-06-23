#!/usr/bin/env roseus

(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(load "models/room73b2-scene.l")
(nextage-init)

(defparameter *toromi-robot* *nextage*)
(defparameter *toromi-ri* *ri*)
(defparameter *stir-arm* :rarm)

;; Units are millimeters and milliseconds unless otherwise noted.
;; TODO: replace with measured cup center in the Nextage waist-local frame.
(defparameter *cup-pos* #f(450 -120 50))
(defparameter *cup-height* 100.0)
(defparameter *cup-depth* 95.0)
(defparameter *cup-top-z-offset* (/ *cup-height* 2.0))

(defparameter *spoon-tip-offset-from-rhand* #f(0 0 0))
(defparameter *stir-radius* 20.0)
(defparameter *stir-depth-from-cup-top* 45.0)
(defparameter *stir-duration* 60000)
(defparameter *stir-step-time* 40)
(defparameter *stir-approach-height* 30.0)
(defparameter *stir-rpy* (list (deg2rad 90) 0 0))
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

(defun rhand-spoon-coords (tip-pos)
  (target-coords-from-tool-tip
   tip-pos
   *stir-rpy*
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
  (send *toromi-robot* :head :angle-vector #f(0 60))
  (send *toromi-robot* :larm :move-end-pos #f(0 0 100))
  (send *toromi-robot* :rarm :move-end-pos #f(0 0 100))
  (when execute
    (send *toromi-ri* :angle-vector (send *toromi-robot* :angle-vector) time)
    (send *toromi-ri* :wait-interpolation))
  (send *toromi-robot* :angle-vector))

(defun pre-stir-tip-pos ()
  (v+ (v+ (cup-top-pos) *pre-stir-right-offset*)
      (float-vector 0 0 *pre-stir-height*)))

(defun stir-tip-pos (theta &optional (z-offset 0.0))
  (let ((center (cup-stir-center-pos)))
    (v+ center
        (float-vector (* *stir-radius* (cos theta))
                      (* *stir-radius* (sin theta))
                      z-offset))))

(defun plan-rarm-stir ()
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

    (solve-rarm-ik (rhand-spoon-coords
                    (stir-tip-pos 0 *stir-approach-height*)))
    (push (current-av) av-seq)
    (push *stir-exit-time* time-seq)

    (list (reverse av-seq) (reverse time-seq))))

(defun rarm-stir-demo (&key (execute t))
  "Move from the initial pose, keep the left arm fixed, and stir for one minute with the right arm."
  (reset-pose :execute nil)
  (let ((plan (plan-rarm-stir)))
    (when execute
      (reset-pose :execute t)
      (execute-av-sequence (car plan) (cadr plan)))
    plan))
