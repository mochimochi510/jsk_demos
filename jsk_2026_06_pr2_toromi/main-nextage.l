#!/usr/bin/env roseus

(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(load "models/room73b2-scene.l")
(nextage-init)

(defparameter *toromi-robot* *nextage*)
(defparameter *toromi-ri* *ri*)
(defparameter *pour-arm* :larm)
(defparameter *stir-arm* :rarm)

;; Units are millimeters and milliseconds unless otherwise noted.
;; TODO: replace with measured cup center in the Nextage waist-local frame.
(defparameter *cup-pos* #f(450 -120 850))
(defparameter *cup-height* 100.0)
(defparameter *cup-depth* 95.0)
(defparameter *cup-radius* 35.0)

(defparameter *cup-top-z-offset* (/ *cup-height* 2.0))
(defparameter *cup-bottom-z-offset* (- *cup-top-z-offset* *cup-depth*))

;; Tool-tip offsets from each gripper/end-coords frame.
;; These are intentionally explicit so only calibration values need to change
;; when porting the motion to another robot or another grasp.
(defparameter *container-mouth-offset-from-lhand* #f(100 0 0))
(defparameter *spoon-tip-offset-from-rhand* #f(180 0 0))

(defparameter *pour-above-cup-height* 55.0)
(defparameter *pour-side-offset* #f(0 25 0))
(defparameter *pour-upright-rpy* (list 0 (deg2rad -95) 0))
(defparameter *pour-tilted-rpy* (list 0 (deg2rad -25) 0))
(defparameter *pour-larm-seed-joint-angles* nil)
(defparameter *pour-approach-time* 3000)
(defparameter *pour-tilt-time* 1500)
(defparameter *pour-hold-time* 3000)

(defparameter *stir-radius* 20.0)
(defparameter *stir-depth-from-cup-top* 45.0)
(defparameter *stir-duration* 60000)
(defparameter *stir-step-time* 40)
(defparameter *stir-approach-height* 60.0)
(defparameter *stir-rpy* (list (deg2rad 90) 0 0))
(defparameter *pre-stir-right-offset* #f(0 -120 0))
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
  "Return end-effector coords that place a tool tip at TIP-POS."
  (send (make-coords :pos tip-pos :rpy rpy)
        :translate
        (scale -1.0 tool-offset)))

(defun lhand-pour-coords (&key (tilted nil))
  (let ((mouth-pos (v+ (v+ (cup-top-pos) *pour-side-offset*)
                       (float-vector 0 0 *pour-above-cup-height*))))
    (target-coords-from-tool-tip
     mouth-pos
     (if tilted *pour-tilted-rpy* *pour-upright-rpy*)
     *container-mouth-offset-from-lhand*)))

(defun rhand-spoon-coords (tip-pos)
  (target-coords-from-tool-tip
   tip-pos
   *stir-rpy*
   *spoon-tip-offset-from-rhand*))

(defun solve-arm-ik (arm target-coords &key (look-at-target t))
  (let ((result (send *toromi-robot* arm
                      :inverse-kinematics
                      target-coords
                      :revert-if-fail nil
                      :look-at-target look-at-target
                      :rotation-axis :z
                      :debug-view nil)))
    (when (boundp '*irtviewer*)
      (send target-coords :draw-on :flush t)
      (send *irtviewer* :draw-objects))
    (unless result
      (error "IK failed for ~A" arm))
    result))

(defun apply-pour-arm-seed-pose ()
  (when (and (eq *pour-arm* :larm) *pour-larm-seed-joint-angles*)
    (send *toromi-robot* :larm :angle-vector
          (apply #'float-vector *pour-larm-seed-joint-angles*))))

(defun solve-pour-arm-ik (target-coords &key (look-at-target t))
  (apply-pour-arm-seed-pose)
  (solve-arm-ik *pour-arm* target-coords :look-at-target look-at-target))

(defun current-av ()
  (send *toromi-robot* :angle-vector))

(defun execute-av-sequence (av-seq time-seq &key (wait t))
  (send *toromi-ri* :angle-vector-sequence av-seq time-seq)
  (when wait (send *toromi-ri* :wait-interpolation)))

(defun ensure-viewer-objects ()
  (when (boundp '*irtviewer*)
    (objects (list *toromi-robot*))
    (send *irtviewer* :draw-objects)))

(defun draw-current-pose ()
  (when (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects)))

(defun show-av-sequence (av-seq &key (sleep-usec 200000))
  "Replay angle-vector sequence on irtviewer without sending commands to the robot."
  (ensure-viewer-objects)
  (dolist (av av-seq)
    (send *toromi-robot* :angle-vector av)
    (draw-current-pose)
    (unix:usleep sleep-usec))
  (send *toromi-robot* :angle-vector))

(defun show-plan (plan &key (sleep-usec 200000))
  "Replay one plan returned by plan-pre-stir, plan-pour-powder, or plan-stir-cup."
  (show-av-sequence (car plan) :sleep-usec sleep-usec))

(defun show-plans (plans &key (sleep-usec 200000))
  "Replay a list of plans such as the value returned by toromi-demo."
  (dolist (plan plans)
    (show-plan plan :sleep-usec sleep-usec))
  (send *toromi-robot* :angle-vector))

(defun show-toromi-demo (&key (sleep-usec 200000))
  "Plan and replay the toromi demo on irtviewer without sending commands to the robot."
  (let ((plans (toromi-demo :execute nil)))
    (show-plans plans :sleep-usec sleep-usec)
    plans))

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
      (float-vector 0 0 *pour-above-cup-height*)))

(defun plan-pre-stir ()
  (solve-arm-ik *stir-arm* (rhand-spoon-coords (pre-stir-tip-pos))
                :look-at-target nil)
  (list (list (current-av)) (list *pre-stir-time*)))

(defun plan-pour-powder ()
  (let (av-seq time-seq)
    (solve-pour-arm-ik (lhand-pour-coords :tilted nil))
    (push (current-av) av-seq)
    (push *pour-approach-time* time-seq)

    (solve-pour-arm-ik (lhand-pour-coords :tilted t))
    (push (current-av) av-seq)
    (push *pour-tilt-time* time-seq)

    ;; Keep the tilted pose so powder can fall into the cup.
    (push (current-av) av-seq)
    (push *pour-hold-time* time-seq)

    (solve-pour-arm-ik (lhand-pour-coords :tilted nil))
    (push (current-av) av-seq)
    (push *pour-tilt-time* time-seq)

    (list (reverse av-seq) (reverse time-seq))))

(defun stir-tip-pos (theta &optional (z-offset 0.0))
  (let ((center (cup-stir-center-pos)))
    (v+ center
        (float-vector (* *stir-radius* (cos theta))
                      (* *stir-radius* (sin theta))
                      z-offset))))

(defun plan-stir-cup ()
  (let ((steps (max 1 (round (/ (float *stir-duration*) *stir-step-time*))))
        av-seq
        time-seq)
    (solve-arm-ik *stir-arm* (rhand-spoon-coords
                         (stir-tip-pos 0 *stir-approach-height*))
                  :look-at-target nil)
    (push (current-av) av-seq)
    (push *stir-approach-time* time-seq)

    (dotimes (i steps)
      (let ((theta (* 2 pi (/ i 12.0))))
        (solve-arm-ik *stir-arm* (rhand-spoon-coords (stir-tip-pos theta))
                      :look-at-target nil)
        (push (current-av) av-seq)
        (push *stir-step-time* time-seq)))

    (solve-arm-ik *stir-arm* (rhand-spoon-coords
                         (stir-tip-pos 0 *stir-approach-height*))
                  :look-at-target nil)
    (push (current-av) av-seq)
    (push *stir-exit-time* time-seq)

    (list (reverse av-seq) (reverse time-seq))))

(defun pour-powder (&key (execute t))
  (let ((plan (plan-pour-powder)))
    (when execute
      (execute-av-sequence (car plan) (cadr plan)))
    plan))

(defun stir-cup (&key (execute t))
  (let ((plan (plan-stir-cup)))
    (when execute
      (execute-av-sequence (car plan) (cadr plan)))
    plan))

(defun pre-stir (&key (execute t))
  (let ((plan (plan-pre-stir)))
    (when execute
      (execute-av-sequence (car plan) (cadr plan)))
    plan))

(defun toromi-demo (&key (execute t))
  "Move the right arm above/right of the cup, pour powder, then stir."
  (reset-pose :execute nil)
  (let ((pre-stir-plan (plan-pre-stir))
        (pour-plan (plan-pour-powder))
        (stir-plan (plan-stir-cup)))
    (when execute
      (reset-pose :execute t)
      (execute-av-sequence (car pre-stir-plan) (cadr pre-stir-plan))
      (execute-av-sequence (car pour-plan) (cadr pour-plan))
      (execute-av-sequence (car stir-plan) (cadr stir-plan)))
    (list pre-stir-plan pour-plan stir-plan)))
