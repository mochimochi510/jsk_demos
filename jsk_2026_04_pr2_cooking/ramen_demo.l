#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus/pr2.l")
(load "models/room73b2-scene.l")
(pr2-init)

(setq *73b2* (room73b2))

(ros::roseus "motomochi")
(ros::load-ros-manifest "jsk_recognition_msgs")

(objects (list *pr2* *73b2*))

(defun apply-angle-vector (filename)
(load (concatenate string "angle_vector/" filename ".l"))
(setq av (symbol-value (intern (string-upcase filename))))
(send *pr2* :angle-vector av)
;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;; (send *ri* :wait-interpolation)
)

;; 腕をしまう
(pr2-tuckarm-pose :larm)
(send *ri* :angle-vector (send *pr2* angle-vector) 3000)
(send *ri* :wait-interpolation)

;; キッチン前(sink0、おそらくIHの前?)の位置を設定
(setq *sink0-model* (make-cascoords :pos #f(1175 1559 300) :rpy (list pi 0 0)))
(setq *sink0* (make-cascoords :pos #f(1175 8459 300) :rpy (list pi 0 0)))

(setq *sink-front* (make-cascoords :pos #f(1355 9350 300) :rpy (list pi 0 0)))
;; (send *ri* :move-to *sink-front* :frame "/map")

;; キッチンの前まで移動
(send *pr2* :move-to *sink0-model* :world) 
;; モデル上で確認。モデル上と実際は、y軸方向に6900mmだけずれている
(send *ri* :move-to *sink0* :frame "/map")

;; 火をつける
(apply-angle-vector "fire-on")

;; 麺が入った皿の上まで右アームを移動
(apply-angle-vector "above-noodle-plate")

;; 麺が入った皿を掴む
(apply-angle-vector "grab-noodle-plate")

;; 鍋の上まで移動
(apply-angle-vector "above-pot")

;; 麺を入れる
(apply-angle-vector "drop-noodle")

;; 麺をほぐす(後で追加)

;; 皿を置く
(apply-angle-vector "put-plate")

;; 皿を離す
(apply-angle-vector "release-plate")

;; 火を止める
(apply-angle-vetor "fire-off")

;; 一旦ここまで?

;; お玉を持つ(アタッチメントの使い方要確認)

;; スープをすくう(鍋の上)

;; スープを皿に入れる

;; 上２つの動作をスープがなくなるくらいまで繰り返す

;; 鍋を掴む

;; 皿の上まで移動

;; 麺を鍋から入れる

;; 鍋を置く
