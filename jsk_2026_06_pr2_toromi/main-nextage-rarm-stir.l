#!/usr/bin/env roseus

(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(ros::roseus-add-msgs "std_msgs")
(load "models/room73b2-scene.l")
(nextage-init)

(defparameter *toromi-robot* *nextage*)
(defparameter *toromi-ri* *ri*)
(defparameter *stir-arm* :rarm)
(defparameter *pour-arm* :larm)
(defparameter *motion-marker-topic* "/toromi/motion_marker")

;; Units are millimeters and milliseconds unless otherwise noted.
;; TODO: replace with measured cup center in the Nextage waist-local frame.
(defparameter *cup-pos* #f(435 -15 215))
(defparameter *cup-height* 100.0)
(defparameter *cup-depth* 95.0)
(defparameter *cup-top-z-offset* (/ *cup-height* 2.0))

(defparameter *spoon-tip-offset-from-rhand* #f(0 0 180))
(defparameter *stir-radius* 8.0)
(defparameter *stir-z-line-half-range* 1.0)
(defparameter *stir-depth-from-cup-top* 105.0)
(defparameter *stir-duration* 120000)
(defparameter *stir-step-time* 20)
(defparameter *stir-approach-height* 30.0)
(defparameter *stir-rpy* (list (deg2rad 90) 0 0))
(defparameter *scoop-tilted-rpy* (list (deg2rad 90) (deg2rad -35) 0))
(defparameter *stir-cycle-steps* 12)
(defparameter *stir-linear-cycle-steps* 14)
(defparameter *stir-pause-time* 3000)
(defparameter *stir-linear-static-time* *stir-pause-time*)
(defparameter *stir-linear-default-move-time* 1000)
(defparameter *stir-repeat-count* 5)
(defparameter *scoop-lift-height* 3.0)
(defparameter *pre-stir-right-offset* #f(0 -70 0))
(defparameter *pre-stir-height* 55.0)
(defparameter *pre-stir-time* 3000)
(defparameter *stir-approach-time* 3000)
(defparameter *stir-start-lower-time* 1000)
(defparameter *stir-start-resync-time* 500)
(defparameter *stir-exit-time* 2000)

;; Positive Y is robot-left. Keep the powder mouth slightly left of the
;; cup center, with the held container protruding 50 mm to robot-right.
(defparameter *powder-pour-mouth-left-offset* #f(0 25 0))
(defparameter *powder-container-mouth-offset-from-lhand* #f(0 -50 0))
(defparameter *powder-pour-level-rpy* (list 0 (deg2rad -95) 0))
(defparameter *powder-pour-tilted-rpy* (list 0 (deg2rad -25) 0))
(defparameter *powder-pour-approach-time* 3000)
(defparameter *powder-pour-tilt-time* 1500)
(defparameter *powder-pour-hold-time* 3000)
(defparameter *powder-pour-return-time* 3000)
(defparameter *powder-pour-after-cycle-count* 2)

(defun robot-local-pos (pos)
  (send *toromi-robot* :transform-vector pos))

(defun cup-top-pos ()
  (v+ (robot-local-pos *cup-pos*) (float-vector 0 0 *cup-top-z-offset*)))

(defun stir-depth-from-offset (stir-depth-offset)
  (- *stir-depth-from-cup-top* stir-depth-offset))

(defun resolve-stir-depth-offset (stir-depth-offset stir-depth-from-cup-top)
  (when (and stir-depth-offset stir-depth-from-cup-top)
    (error "Specify only one of stir-depth-offset or stir-depth-from-cup-top."))
  (if stir-depth-offset
      stir-depth-offset
    (if stir-depth-from-cup-top
        stir-depth-from-cup-top
      0.0)))

(defun cup-stir-center-pos
  (&optional (stir-depth-offset 0.0))
  (v+ (cup-top-pos)
      (float-vector 0 0 (- (stir-depth-from-offset stir-depth-offset)))))

(defun target-coords-from-tool-tip (tip-pos rpy tool-offset)
  (send (make-coords :pos tip-pos :rpy rpy)
        :translate
        (scale -1.0 tool-offset)))

(defun rhand-spoon-coords (tip-pos &optional (rpy *stir-rpy*))
  (target-coords-from-tool-tip
   tip-pos
   rpy
   *spoon-tip-offset-from-rhand*))

(defun lhand-powder-pour-mouth-pos ()
  (v+ (v+ (cup-top-pos) *powder-pour-mouth-left-offset*)
      (float-vector 0 0 *pre-stir-height*)))

(defun lhand-powder-pour-coords (&key (tilted nil))
  (target-coords-from-tool-tip
   (lhand-powder-pour-mouth-pos)
   (if tilted *powder-pour-tilted-rpy* *powder-pour-level-rpy*)
   *powder-container-mouth-offset-from-lhand*))

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

(defun solve-larm-powder-pour-ik (target-coords)
  (let ((result (send *toromi-robot* *pour-arm*
                      :inverse-kinematics
                      target-coords
                      :revert-if-fail nil
                      :look-at-target t
                      :rotation-axis :z
                      :debug-view nil)))
    (unless result
      (error "IK failed for ~A" *pour-arm*))
    result))

(defun current-av ()
  (send *toromi-robot* :angle-vector))

(defun execute-av-sequence (av-seq time-seq &key (wait t))
  (send *toromi-ri* :angle-vector-sequence av-seq time-seq)
  (when wait (send *toromi-ri* :wait-interpolation)))

(defun wait-before-stir ()
  (format t "~%Reached stir start position. Waiting ~A ms before stirring.~%"
          *stir-pause-time*)
  (unix:usleep (* *stir-pause-time* 1000)))

(defun wait-before-lift ()
  (format t "~%Finished stirring. Waiting 5 seconds before lifting the spoon.~%")
  (unix:sleep 3))

(defun stir-start-approach-times (time-seq)
  (let ((approach-times (subseq time-seq 0 3)))
    (setf (nth 2 approach-times) *stir-start-lower-time*)
    approach-times))

(defun resync-stir-start-pose (plan)
  (let ((start-av (nth 2 (car plan))))
    (send *toromi-robot* :angle-vector start-av)
    (send *toromi-ri* :angle-vector start-av *stir-start-resync-time*)
    (send *toromi-ri* :wait-interpolation)))

(defun init-motion-marker-publisher ()
  (unless (ros::get-topic-publisher *motion-marker-topic*)
    (ros::advertise *motion-marker-topic* std_msgs::Header 10)))

(defun publish-motion-marker (event-name)
  (init-motion-marker-publisher)
  (let ((stamp (ros::time-now)))
    (dotimes (i 5)
      (ros::publish
       *motion-marker-topic*
       (instance std_msgs::Header :init
                 :stamp stamp
                 :frame_id event-name))
      (unix:usleep 10000))
    stamp))

(defun cycle-start-event-name (cycle-number)
  (cond
   ((< cycle-number 10)
    (format nil "cycle_start_000~D" cycle-number))
   ((< cycle-number 100)
    (format nil "cycle_start_00~D" cycle-number))
   ((< cycle-number 1000)
    (format nil "cycle_start_0~D" cycle-number))
   (t
    (format nil "cycle_start_~D" cycle-number))))

(defun reset-pose (&key (execute t) (time 3000))
  (send *toromi-robot* :reset-pose)
  (send *toromi-robot* :head :angle-vector #f(0 30))
  (send *toromi-robot* :larm :move-end-pos #f(0 0 150))
  (send *toromi-robot* :rarm :move-end-pos #f(0 0 150))
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

(defun stir-tip-pos
  (theta &optional (z-offset 0.0) (stir-depth-offset 0.0))
  (let ((center (cup-stir-center-pos stir-depth-offset)))
    (v+ center
        (float-vector (* *stir-radius* (cos theta))
                      (* *stir-radius* (sin theta))
                      z-offset))))

(defun stir-tip-pos-from-offset
  (x y z &optional (z-offset 0.0) (stir-depth-offset 0.0))
  (v+ (cup-stir-center-pos stir-depth-offset)
      (float-vector x y (+ z z-offset))))

(defun line-motion-p (motion)
  (memq motion (list :x-line :y-line :z-line)))

(defun stir-motion-cycle-steps (motion)
  (if (line-motion-p motion)
      *stir-linear-cycle-steps*
    *stir-cycle-steps*))

(defun stir-motion-cycle-time (motion)
  (* *stir-cycle-steps* *stir-step-time*))

(defun positive-number (value name)
  (when (<= value 0)
    (error "~A must be > 0: ~A" name value))
  value)

(defun integer-duration (value name)
  (round (positive-number value name)))

(defun split-duration (duration count)
  (let* ((total (integer-duration duration "duration"))
         (base-time (floor (/ total (float count))))
         (extra-count (mod total count))
         time-seq)
    (dotimes (i count)
      (push (+ base-time (if (< i extra-count) 1 0)) time-seq))
    (reverse time-seq)))

(defun number-list-sum (numbers)
  (let ((rest numbers)
        (sum 0))
    (while rest
      (setq sum (+ sum (car rest)))
      (setq rest (cdr rest)))
    sum))

(defun plan-larm-powder-pour (start-larm-av)
  (let (av-seq time-seq)
    (solve-larm-powder-pour-ik (lhand-powder-pour-coords :tilted nil))
    (push (send *toromi-robot* *pour-arm* :angle-vector) av-seq)
    (push *powder-pour-approach-time* time-seq)

    (solve-larm-powder-pour-ik (lhand-powder-pour-coords :tilted t))
    (push (send *toromi-robot* *pour-arm* :angle-vector) av-seq)
    (push *powder-pour-tilt-time* time-seq)

    (push (send *toromi-robot* *pour-arm* :angle-vector) av-seq)
    (push *powder-pour-hold-time* time-seq)

    (solve-larm-powder-pour-ik (lhand-powder-pour-coords :tilted nil))
    (push (send *toromi-robot* *pour-arm* :angle-vector) av-seq)
    (push *powder-pour-tilt-time* time-seq)

    (send *toromi-robot* *pour-arm* :angle-vector start-larm-av)
    (push (send *toromi-robot* *pour-arm* :angle-vector) av-seq)
    (push *powder-pour-return-time* time-seq)

    (list (reverse av-seq) (reverse time-seq))))

(defun interpolate-angle-vector (start-av goal-av ratio)
  (v+ start-av (scale ratio (v- goal-av start-av))))

(defun larm-powder-pour-av-at-time (start-larm-av pour-plan elapsed-time)
  (let ((from-av start-larm-av)
        (target-avs (car pour-plan))
        (target-times (cadr pour-plan))
        (remaining-time elapsed-time))
    (while (and target-avs target-times (> remaining-time (car target-times)))
      (setq remaining-time (- remaining-time (car target-times)))
      (setq from-av (car target-avs))
      (setq target-avs (cdr target-avs))
      (setq target-times (cdr target-times)))
    (if (and target-avs target-times)
        (interpolate-angle-vector
         from-av
         (car target-avs)
         (min 1.0 (/ (float remaining-time) (car target-times))))
      from-av)))

(defun replace-larm-in-angle-vector (base-av larm-av)
  (send *toromi-robot* :angle-vector base-av)
  (send *toromi-robot* *pour-arm* :angle-vector larm-av)
  (current-av))

(defun add-larm-powder-pour-to-circle-plan (plan)
  (let* ((av-seq (car plan))
         (time-seq (cadr plan))
         ;; pre-stir, approach, and the synchronized stir start pose.
         (stir-start-index 3)
         (pour-start-index
          (+ stir-start-index
             (* *powder-pour-after-cycle-count* *stir-cycle-steps*))))
    (when (<= (length av-seq) pour-start-index)
      (error "Circle continuous stir must continue past ~A cycles before powder pour."
             *powder-pour-after-cycle-count*))
    (send *toromi-robot* :angle-vector (nth (- pour-start-index 1) av-seq))
    (let* ((start-larm-av (send *toromi-robot* *pour-arm* :angle-vector))
           (pour-plan (plan-larm-powder-pour start-larm-av))
           (pour-total-time (number-list-sum (cadr pour-plan)))
           (available-time (number-list-sum (subseq time-seq pour-start-index))))
      (when (< available-time pour-total-time)
        (error "Circle continuous stir must continue at least ~A ms after the powder pour starts."
               pour-total-time))
      (let ((new-av-seq nil)
            (index 0)
            (elapsed-time 0)
            (avs av-seq)
            (times time-seq))
        (while avs
          (let ((base-av (car avs))
                (step-time (car times)))
            (if (< index pour-start-index)
                (push base-av new-av-seq)
              (progn
                (setq elapsed-time (+ elapsed-time step-time))
                (if (<= elapsed-time pour-total-time)
                    (push
                     (replace-larm-in-angle-vector
                      base-av
                      (larm-powder-pour-av-at-time
                       start-larm-av pour-plan elapsed-time))
                     new-av-seq)
                  (push base-av new-av-seq)))))
          (setq index (+ index 1))
          (setq avs (cdr avs))
          (setq times (cdr times)))
        (list (reverse new-av-seq) time-seq)))))

(defun stir-linear-half-range (motion)
  (if (eq motion :z-line)
      *stir-z-line-half-range*
    *stir-radius*))

(defun stir-linear-full-range (motion)
  (* 2.0 (stir-linear-half-range motion)))

(defun stir-linear-leg-steps ()
  (let ((movable-steps (- *stir-linear-cycle-steps* 2)))
    (unless (= (mod movable-steps 2) 0)
      (error "*stir-linear-cycle-steps* - 2 must be even: ~A"
             *stir-linear-cycle-steps*))
    (floor (/ movable-steps 2))))

(defun stir-linear-negative-end-index ()
  (stir-linear-leg-steps))

(defun stir-linear-negative-static-index ()
  (+ (stir-linear-negative-end-index) 1))

(defun resolve-linear-move-time (motion linear-speed linear-move-time)
  (when (and linear-speed linear-move-time)
    (error "Specify only one of linear-speed or linear-move-time."))
  (cond
   (linear-move-time
    (integer-duration linear-move-time "linear-move-time"))
   (linear-speed
    (integer-duration
     (* 1000.0 (/ (stir-linear-full-range motion)
                  (positive-number linear-speed "linear-speed")))
     "linear-speed move time"))
   (t
    (integer-duration
     *stir-linear-default-move-time* "*stir-linear-default-move-time*"))))

(defun linear-stir-motion-step-times
  (motion linear-speed linear-move-time)
  (let ((move-time (resolve-linear-move-time
                    motion linear-speed linear-move-time))
        (static-time (integer-duration
                      *stir-linear-static-time* "*stir-linear-static-time*"))
        (leg-steps (stir-linear-leg-steps)))
    (append
     (list static-time)
     (split-duration move-time leg-steps)
     (list static-time)
     (split-duration move-time leg-steps))))

(defun stir-motion-step-times
  (motion &key (linear-speed nil)
               (linear-move-time nil))
  (cond
   ((line-motion-p motion)
    (linear-stir-motion-step-times motion linear-speed linear-move-time))
   ((or linear-speed linear-move-time)
    (error "linear-speed/linear-move-time are only valid for line motions: ~A" motion))
   (t
    (let* ((cycle-steps (stir-motion-cycle-steps motion))
           (cycle-time (stir-motion-cycle-time motion))
           (base-time (floor (/ cycle-time (float cycle-steps))))
           (extra-count (mod cycle-time cycle-steps)))
      (let (time-seq)
        (dotimes (i cycle-steps)
          (push (+ base-time (if (< i extra-count) 1 0)) time-seq))
        (reverse time-seq))))))

(defun stir-linear-offset (cycle-i &optional (half-range *stir-radius*))
  (let ((negative-end-i (stir-linear-negative-end-index))
        (negative-static-i (stir-linear-negative-static-index)))
    (cond
     ((= cycle-i 0)
      half-range)
     ((<= cycle-i negative-end-i)
      (+ half-range
         (* -2.0 half-range
            (/ cycle-i (float negative-end-i)))))
     ((= cycle-i negative-static-i)
      (- half-range))
     (t
      (+ (- half-range)
         (* 2.0 half-range
            (/ (- cycle-i negative-static-i)
               (float (stir-linear-leg-steps)))))))))

(defun line-motion-name (motion)
  (cond
   ((eq motion :x-line) "x_line")
   ((eq motion :y-line) "y_line")
   ((eq motion :z-line) "z_line")
   (t nil)))

(defun stir-linear-stage-name (cycle-i)
  (cond
   ((= cycle-i 0) "positive_endpoint_static")
   ((<= cycle-i (stir-linear-negative-end-index))
    "move_to_negative_endpoint")
   ((= cycle-i (stir-linear-negative-static-index))
    "negative_endpoint_static")
   (t "move_to_positive_endpoint")))

(defun stir-linear-stage-event (motion cycle-i)
  (let ((motion-name (line-motion-name motion)))
    (when motion-name
      (format nil "~A_~A" motion-name (stir-linear-stage-name cycle-i)))))

(defun next-stir-cycle-index (stir-i cycle-steps)
  (let ((cycle-i (mod stir-i cycle-steps)))
    (if (= cycle-i 0)
        (+ stir-i cycle-steps)
      (+ stir-i (- cycle-steps cycle-i)))))

(defun stir-marker-chunk-end-index (motion stir-i end-stir-i)
  (let* ((cycle-steps (stir-motion-cycle-steps motion))
         (cycle-end (min end-stir-i
                         (next-stir-cycle-index stir-i cycle-steps))))
    (if (line-motion-p motion)
        (let ((event (stir-linear-stage-event motion (mod stir-i cycle-steps)))
              (j (+ stir-i 1)))
          (while (and (< j cycle-end)
                      (equal event
                             (stir-linear-stage-event motion
                                                      (mod j cycle-steps))))
            (setq j (+ j 1)))
          j)
      cycle-end)))

(defun continuous-cycle-start-p (stir-i first-stir-i cycle-steps)
  (or (= stir-i first-stir-i)
      (= (mod stir-i cycle-steps) 0)))

(defun continuous-cycle-number (stir-i first-stir-i cycle-steps)
  (if (= stir-i first-stir-i)
      1
    (+ (floor (/ stir-i cycle-steps)) 1)))

(defun publish-initial-linear-state-marker (motion)
  (when (line-motion-p motion)
    (publish-motion-marker (stir-linear-stage-event motion 0))))

(defun execute-stir-motion-with-markers
  (av-seq time-seq motion mode &optional (first-stir-i 1))
  (if (and (not (line-motion-p motion))
           (not (eq mode :continuous)))
      (execute-av-sequence av-seq time-seq)
    (let* ((cycle-steps (stir-motion-cycle-steps motion))
           (end-stir-i (+ first-stir-i (length av-seq)))
           (stir-i first-stir-i))
      (while (< stir-i end-stir-i)
        (let* ((seq-start (- stir-i first-stir-i))
               (chunk-end (stir-marker-chunk-end-index motion stir-i end-stir-i))
               (seq-end (- chunk-end first-stir-i)))
          (when (and (eq mode :continuous)
                     (continuous-cycle-start-p stir-i first-stir-i cycle-steps))
            (publish-motion-marker
             (cycle-start-event-name
              (continuous-cycle-number stir-i first-stir-i cycle-steps))))
          (when (line-motion-p motion)
            (publish-motion-marker
             (stir-linear-stage-event motion (mod stir-i cycle-steps))))
          (execute-av-sequence
           (subseq av-seq seq-start seq-end)
           (subseq time-seq seq-start seq-end))
          (setq stir-i chunk-end))))))

(defun final-linear-positive-endpoint-pause-p
  (motion stir-count &optional (first-stir-i 1))
  (when (and (line-motion-p motion) (> stir-count 0))
    (let* ((cycle-steps (stir-motion-cycle-steps motion))
           (last-stir-i (+ first-stir-i stir-count -1)))
      (= (mod last-stir-i cycle-steps) (- cycle-steps 1)))))

(defun execute-final-linear-endpoint-pause
  (av-seq motion &optional (first-stir-i 1))
  (when (final-linear-positive-endpoint-pause-p
         motion (length av-seq) first-stir-i)
    (publish-motion-marker (stir-linear-stage-event motion 0))
    (execute-av-sequence
     (list (car (last av-seq)))
     (list *stir-linear-static-time*))))

(defun stir-motion-target
  (motion i &optional (z-offset 0.0) (stir-depth-offset 0.0))
  (let* ((cycle-steps (stir-motion-cycle-steps motion))
         (cycle-i (mod i cycle-steps))
         (theta (* 2 pi (/ cycle-i (float cycle-steps)))))
    (cond
     ((eq motion :circle)
      (list (stir-tip-pos theta z-offset stir-depth-offset) *stir-rpy*))
     ((eq motion :reverse-circle)
      (list (stir-tip-pos (- theta) z-offset stir-depth-offset) *stir-rpy*))
     ((eq motion :x-line)
      (list (stir-tip-pos-from-offset
             (stir-linear-offset cycle-i) 0 0 z-offset stir-depth-offset)
            *stir-rpy*))
     ((eq motion :y-line)
      (list (stir-tip-pos-from-offset
             0 (stir-linear-offset cycle-i) 0 z-offset stir-depth-offset)
            *stir-rpy*))
     ((eq motion :z-line)
      (list (stir-tip-pos-from-offset
             0 0 (stir-linear-offset cycle-i *stir-z-line-half-range*)
             z-offset stir-depth-offset)
            *stir-rpy*))
     ((eq motion :scoop)
      (cond
       ((< cycle-i 5)
        (list (stir-tip-pos-from-offset
               0 0
               (- *scoop-lift-height*
                  (* 2 *scoop-lift-height* (/ cycle-i 4.0)))
               z-offset stir-depth-offset)
              *stir-rpy*))
       ((< cycle-i 9)
        (list (stir-tip-pos-from-offset
               0 0
               (+ (- *scoop-lift-height*)
                  (* 2 *scoop-lift-height* (/ (- cycle-i 4) 4.0)))
               z-offset stir-depth-offset)
              *stir-rpy*))
       ((< cycle-i 11)
        (list (stir-tip-pos-from-offset
               0 0 *scoop-lift-height* z-offset stir-depth-offset)
              *scoop-tilted-rpy*))
       (t
        (list (stir-tip-pos-from-offset
               0 0 *scoop-lift-height* z-offset stir-depth-offset)
              *stir-rpy*))))
     (t
      (error "Unknown stir motion: ~A" motion)))))

(defun continuous-stir-steps (motion repeat-count &key (step-times nil))
  (if repeat-count
      (progn
        (when (< repeat-count 1)
          (error "repeat-count must be >= 1: ~A" repeat-count))
        (* (round repeat-count) (stir-motion-cycle-steps motion)))
    (let* ((cycle-steps (stir-motion-cycle-steps motion))
           (cycle-time (if step-times
                           (number-list-sum step-times)
                         (stir-motion-cycle-time motion)))
           (average-step-time (/ (float cycle-time) cycle-steps)))
      (max 1 (round (/ (float *stir-duration*) average-step-time))))))

(defun plan-rarm-stir
  (&key (motion :circle) (mode :continuous) (repeat-count nil)
        (linear-speed nil) (linear-move-time nil)
        (stir-depth-offset nil) (stir-depth-from-cup-top nil))
  (let ((depth-offset (resolve-stir-depth-offset
                       stir-depth-offset stir-depth-from-cup-top))
        av-seq
        time-seq)
    (solve-rarm-ik (rhand-spoon-coords (pre-stir-tip-pos)))
    (push (current-av) av-seq)
    (push *pre-stir-time* time-seq)

    (let ((target (stir-motion-target
                   motion 0 *stir-approach-height* depth-offset)))
      (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
      (push (current-av) av-seq)
      (push *stir-approach-time* time-seq))

    (let ((step-times (stir-motion-step-times
                       motion
                       :linear-speed linear-speed
                       :linear-move-time linear-move-time)))
      (cond
       ((eq mode :continuous)
        (dotimes (i (continuous-stir-steps
                     motion repeat-count :step-times step-times))
          (let ((target (stir-motion-target motion i 0.0 depth-offset)))
            (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
            (push (current-av) av-seq)
            (push (nth (mod i (length step-times)) step-times) time-seq))))
       ((eq mode :once)
        (dotimes (i (stir-motion-cycle-steps motion))
          (let ((target (stir-motion-target motion i 0.0 depth-offset)))
            (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
            (push (current-av) av-seq)
            (push (nth i step-times) time-seq))))
       ((eq mode :five)
        (dotimes (j *stir-repeat-count*)
          (dotimes (i (stir-motion-cycle-steps motion))
            (let ((target (stir-motion-target motion i 0.0 depth-offset)))
              (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
              (push (current-av) av-seq)
              (push (nth i step-times) time-seq)))))
       (t
        (error "Unknown stir mode: ~A" mode))))

    #|
    (let ((target (stir-motion-target motion 0 *stir-approach-height*)))
      (solve-rarm-ik (rhand-spoon-coords (car target) (cadr target)))
      (push (current-av) av-seq)
      (push *stir-exit-time* time-seq))
    |#

    (list (reverse av-seq) (reverse time-seq))))

(defun plan-rarm-stir-circle
  (&key (repeat-count nil)
        (stir-depth-offset nil) (stir-depth-from-cup-top nil))
  (let ((steps (continuous-stir-steps :circle repeat-count))
        (depth-offset (resolve-stir-depth-offset
                       stir-depth-offset stir-depth-from-cup-top))
        av-seq
        time-seq)
    (solve-rarm-ik (rhand-spoon-coords (pre-stir-tip-pos)))
    (push (current-av) av-seq)
    (push *pre-stir-time* time-seq)

    (solve-rarm-ik (rhand-spoon-coords
                    (stir-tip-pos
                     0 *stir-approach-height* depth-offset)))
    (push (current-av) av-seq)
    (push *stir-approach-time* time-seq)

    (dotimes (i steps)
      (let ((theta (* 2 pi (/ i 12.0))))
        (solve-rarm-ik
         (rhand-spoon-coords
          (stir-tip-pos theta 0.0 depth-offset)))
        (push (current-av) av-seq)
        (push *stir-step-time* time-seq)))

    #|
    (solve-rarm-ik (rhand-spoon-coords
                    (stir-tip-pos 0 *stir-approach-height*)))
    (push (current-av) av-seq)
    (push *stir-exit-time* time-seq)
    |#

    (add-larm-powder-pour-to-circle-plan
     (list (reverse av-seq) (reverse time-seq)))))

(defun rarm-stir-demo
  (&key (execute t) (motion :circle) (mode :continuous) (repeat-count nil)
        (linear-speed nil) (linear-move-time nil)
        (stir-depth-offset nil) (stir-depth-from-cup-top nil))
  "Move from the initial pose and stir with the right arm. In circle continuous mode, pour powder with the left arm after two cycles while stirring continues."
  (when (and (not (line-motion-p motion))
             (or linear-speed linear-move-time))
    (error "linear-speed/linear-move-time are only valid for line motions: ~A" motion))
  (let ((depth-offset (resolve-stir-depth-offset
                       stir-depth-offset stir-depth-from-cup-top)))
    (reset-pose :execute nil)
    (let ((plan (if (and (eq motion :circle) (eq mode :continuous))
                    (plan-rarm-stir-circle
                     :repeat-count repeat-count
                     :stir-depth-offset depth-offset)
                  (plan-rarm-stir
                   :motion motion
                   :mode mode
                   :repeat-count repeat-count
                   :linear-speed linear-speed
                   :linear-move-time linear-move-time
                   :stir-depth-offset depth-offset))))
    (when execute
      (reset-pose :execute t)
      (publish-motion-marker "rarm_stir_prepare_start")
      (execute-av-sequence (subseq (car plan) 0 3) (stir-start-approach-times (cadr plan)))
      (publish-motion-marker "rarm_stir_pause_start")
      (publish-initial-linear-state-marker motion)
      (publish-motion-marker "baseline_start")
      (wait-before-stir)
      (resync-stir-start-pose plan)
      (publish-motion-marker "baseline_end")
      (publish-motion-marker "rarm_stir_start")
      (let ((stir-av-seq (subseq (car plan) 3))
            (stir-time-seq (subseq (cadr plan) 3)))
        (execute-stir-motion-with-markers
         stir-av-seq
         stir-time-seq
         motion
         mode)
        (execute-final-linear-endpoint-pause stir-av-seq motion))
      (publish-motion-marker "rarm_stir_motion_end")
      (publish-motion-marker "rarm_stir_lift_pause_start")
      (wait-before-lift)
      (send *toromi-robot* :angle-vector (car (last (car plan))))
      (finish-rarm-stir)
      (publish-motion-marker "rarm_stir_finish_end"))
    plan)))
