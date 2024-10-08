(define (domain blocks-domain)
    (:requirements :conditional-effects :derived-predicates :disjunctive-preconditions :negative-preconditions :strips :typing)
    (:types block)
    (:predicates (Y-Oon_b1_b2-and-Oon_b6_b1) (Y-Oon_b2_b3-and-Oon_b6_b1) (Y-Oon_b3_b4-and-Oon_b6_b1) (Y-Oon_b6_b1) (clear ?x - block)  (emptyhand) (holding ?x - block)  (on ?x - block ?y - block)  (ontable ?x - block)  (val-Oon_b1_b2-and-Oon_b6_b1) (val-Oon_b1_b2-and-Oon_b6_b1-and-Oon_b2_b3-and-Oon_b6_b1-and-Oon_b3_b4-and-Oon_b6_b1) (val-Oon_b2_b3-and-Oon_b6_b1) (val-Oon_b3_b4-and-Oon_b6_b1) (val-Oon_b6_b1) (val-on_b1_b2) (val-on_b1_b2-and-Oon_b6_b1) (val-on_b2_b3) (val-on_b2_b3-and-Oon_b6_b1) (val-on_b3_b4) (val-on_b3_b4-and-Oon_b6_b1) (val-on_b6_b1))
    (:derived (val-Oon_b1_b2-and-Oon_b6_b1) (or (val-on_b1_b2-and-Oon_b6_b1) (Y-Oon_b1_b2-and-Oon_b6_b1)))
     (:derived (val-Oon_b1_b2-and-Oon_b6_b1-and-Oon_b2_b3-and-Oon_b6_b1-and-Oon_b3_b4-and-Oon_b6_b1) (and (val-Oon_b1_b2-and-Oon_b6_b1) (val-Oon_b2_b3-and-Oon_b6_b1) (val-Oon_b3_b4-and-Oon_b6_b1)))
     (:derived (val-Oon_b2_b3-and-Oon_b6_b1) (or (val-on_b2_b3-and-Oon_b6_b1) (Y-Oon_b2_b3-and-Oon_b6_b1)))
     (:derived (val-Oon_b3_b4-and-Oon_b6_b1) (or (val-on_b3_b4-and-Oon_b6_b1) (Y-Oon_b3_b4-and-Oon_b6_b1)))
     (:derived (val-Oon_b6_b1) (or (val-on_b6_b1) (Y-Oon_b6_b1)))
     (:derived (val-Oon_b6_b1) (or (val-on_b6_b1) (Y-Oon_b6_b1)))
     (:derived (val-Oon_b6_b1) (or (val-on_b6_b1) (Y-Oon_b6_b1)))
     (:derived (val-on_b1_b2) (on b1 b2))
     (:derived (val-on_b1_b2-and-Oon_b6_b1) (and (val-on_b1_b2) (val-Oon_b6_b1)))
     (:derived (val-on_b2_b3) (on b2 b3))
     (:derived (val-on_b2_b3-and-Oon_b6_b1) (and (val-on_b2_b3) (val-Oon_b6_b1)))
     (:derived (val-on_b3_b4) (on b3 b4))
     (:derived (val-on_b3_b4-and-Oon_b6_b1) (and (val-on_b3_b4) (val-Oon_b6_b1)))
     (:derived (val-on_b6_b1) (on b6 b1))
     (:derived (val-on_b6_b1) (on b6 b1))
     (:derived (val-on_b6_b1) (on b6 b1))
    (:action pick-up
        :parameters (?x - block)
        :precondition (and (clear ?x) (ontable ?x) (emptyhand))
        :effect (and (not (ontable ?x)) (not (clear ?x)) (not (emptyhand)) (holding ?x) (when (not (val-Oon_b6_b1)) (not (Y-Oon_b6_b1))) (when (val-Oon_b6_b1) (Y-Oon_b6_b1)) (when (not (val-Oon_b1_b2-and-Oon_b6_b1)) (not (Y-Oon_b1_b2-and-Oon_b6_b1))) (when (val-Oon_b3_b4-and-Oon_b6_b1) (Y-Oon_b3_b4-and-Oon_b6_b1)) (when (not (val-Oon_b2_b3-and-Oon_b6_b1)) (not (Y-Oon_b2_b3-and-Oon_b6_b1))) (when (not (val-Oon_b3_b4-and-Oon_b6_b1)) (not (Y-Oon_b3_b4-and-Oon_b6_b1))) (when (val-Oon_b2_b3-and-Oon_b6_b1) (Y-Oon_b2_b3-and-Oon_b6_b1)) (when (val-Oon_b1_b2-and-Oon_b6_b1) (Y-Oon_b1_b2-and-Oon_b6_b1)))
    )
     (:action put-down
        :parameters (?x - block)
        :precondition (holding ?x)
        :effect (and (not (holding ?x)) (clear ?x) (emptyhand) (ontable ?x) (when (not (val-Oon_b6_b1)) (not (Y-Oon_b6_b1))) (when (val-Oon_b6_b1) (Y-Oon_b6_b1)) (when (not (val-Oon_b1_b2-and-Oon_b6_b1)) (not (Y-Oon_b1_b2-and-Oon_b6_b1))) (when (val-Oon_b3_b4-and-Oon_b6_b1) (Y-Oon_b3_b4-and-Oon_b6_b1)) (when (not (val-Oon_b2_b3-and-Oon_b6_b1)) (not (Y-Oon_b2_b3-and-Oon_b6_b1))) (when (not (val-Oon_b3_b4-and-Oon_b6_b1)) (not (Y-Oon_b3_b4-and-Oon_b6_b1))) (when (val-Oon_b2_b3-and-Oon_b6_b1) (Y-Oon_b2_b3-and-Oon_b6_b1)) (when (val-Oon_b1_b2-and-Oon_b6_b1) (Y-Oon_b1_b2-and-Oon_b6_b1)))
    )
     (:action stack
        :parameters (?x - block ?y - block)
        :precondition (and (holding ?x) (clear ?y))
        :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (emptyhand) (on ?x ?y) (when (not (val-Oon_b6_b1)) (not (Y-Oon_b6_b1))) (when (val-Oon_b6_b1) (Y-Oon_b6_b1)) (when (not (val-Oon_b1_b2-and-Oon_b6_b1)) (not (Y-Oon_b1_b2-and-Oon_b6_b1))) (when (val-Oon_b3_b4-and-Oon_b6_b1) (Y-Oon_b3_b4-and-Oon_b6_b1)) (when (not (val-Oon_b2_b3-and-Oon_b6_b1)) (not (Y-Oon_b2_b3-and-Oon_b6_b1))) (when (not (val-Oon_b3_b4-and-Oon_b6_b1)) (not (Y-Oon_b3_b4-and-Oon_b6_b1))) (when (val-Oon_b2_b3-and-Oon_b6_b1) (Y-Oon_b2_b3-and-Oon_b6_b1)) (when (val-Oon_b1_b2-and-Oon_b6_b1) (Y-Oon_b1_b2-and-Oon_b6_b1)))
    )
     (:action unstack
        :parameters (?x - block ?y - block)
        :precondition (and (on ?x ?y) (clear ?x) (emptyhand))
        :effect (and (holding ?x) (clear ?y) (not (clear ?x)) (not (emptyhand)) (not (on ?x ?y)) (when (not (val-Oon_b6_b1)) (not (Y-Oon_b6_b1))) (when (val-Oon_b6_b1) (Y-Oon_b6_b1)) (when (not (val-Oon_b1_b2-and-Oon_b6_b1)) (not (Y-Oon_b1_b2-and-Oon_b6_b1))) (when (val-Oon_b3_b4-and-Oon_b6_b1) (Y-Oon_b3_b4-and-Oon_b6_b1)) (when (not (val-Oon_b2_b3-and-Oon_b6_b1)) (not (Y-Oon_b2_b3-and-Oon_b6_b1))) (when (not (val-Oon_b3_b4-and-Oon_b6_b1)) (not (Y-Oon_b3_b4-and-Oon_b6_b1))) (when (val-Oon_b2_b3-and-Oon_b6_b1) (Y-Oon_b2_b3-and-Oon_b6_b1)) (when (val-Oon_b1_b2-and-Oon_b6_b1) (Y-Oon_b1_b2-and-Oon_b6_b1)))
    )
)