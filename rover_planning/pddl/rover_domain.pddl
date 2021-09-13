(define (domain rover)
  (:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

 (:types points rover camera)

;; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;

(:predicates
       (point ?x - points)
       (rover ?r - rover)
       (camera ?c - camera)
       (move-possible ?x - points ?y - points)
       (at ?x - points)
       (photography ?x - points ?c - camera)
       (recording ?x - points ?c - camera)
       (calibration ?x - points ?r -rover)
       (free ?r - rover)
)

;; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;

(:functions
      (distance ?x -points ?y - points)
      (battery ?r - rover)
      (consum ?r - rover)
      (velocity ?r - rover)
      (capacity ?r - rover)
      (recharge-rate ?r - rover)
      (photo-energy ?r - rover)
      (calibration-energy ?r - rover)
)

;; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?x ?y - points ?r -rover)
    :duration (= ?duration (/ (distance ?x ?y) (velocity ?r)))
    :condition (and (at start (at ?x))
                    (at start (move-possible ?x ?y))
                    (at start (>= (battery ?r) (* (distance ?x ?y)(consum ?r))))
                    (at start (free ?r))
               )
    :effect (and (at end (at ?y))
                 (at start (not (free ?r)))
                 (at start (not (at ?x)))
                 (at end (decrease (battery ?r)(* (distance ?x ?y)(consum ?r))))
                 (at end (free ?r))
            )
)

(:durative-action recharge
    :parameters (?r - rover ?x ?y - points)
    :duration ( = ?duration (/ (- (capacity ?r) (battery ?r)) (recharge-rate ?r)))
    :condition (and (at start (< (battery ?r) (* (distance ?x ?y) (consum ?r))))
                    (at start (free ?r))
                    (at start (at ?x))
               )
    :effect (and (at start (not (free ?r)))
                 (at end (assign (battery ?r) (capacity ?r)))
                 (at end (free ?r))
            )
)

(:durative-action photo
    :parameters (?x - points ?c - camera ?r - rover)
    :duration ( = ?duration 7)
    :condition (and (at start (at ?x))
                    (at start (free ?r))
               )
    :effect (and (at start (photography ?x ?c))
                 (at start (not (free ?r)))
                 (at end (decrease (battery ?r)(photo-energy ?r)))
                 (at end (free ?r))
            )
)

(:durative-action record
    :parameters (?x - points ?c - camera ?r - rover)
    :duration ( = ?duration 7)
    :condition (and (at start (at ?x))
                    (at start (free ?r))
               )
    :effect (and (at start (recording ?x ?c))
                 (at start (not (free ?r)))
                 ;(at end (decrease (battery ?r)(photo-energy ?r)))
                 (at end (free ?r))
            )
)

(:durative-action calibration
    :parameters (?x - points ?r - rover)
    :duration ( = ?duration 7)
    :condition (and (at start (at ?x))
                    (at start (free ?r))
               )
    :effect (and (at start (calibration ?x ?r))
                 (at start (not (free ?r)))
                 (at end (decrease (battery ?r)(calibration-energy ?r)))
                 (at end (free ?r))
            )
)


)
;; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
