(define (domain blocks-domain)
    (:requirements :conditional-effects :derived-predicates :disjunctive-preconditions :negative-preconditions :strips :typing)
    (:types block)
    (:predicates (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (Y-Oon_b1_b2) (Y-Oon_b2_b3) (Y-Oon_b3_b4) (Y-Oon_b4_b5) (clear ?x - block)  (emptyhand) (holding ?x - block)  (on ?x - block ?y - block)  (ontable ?x - block)  (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (val-Oon_b1_b2) (val-Oon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (val-Oon_b2_b3) (val-Oon_b3_b4) (val-Oon_b4_b5) (val-on_b1_b2) (val-on_b2_b3) (val-on_b3_b4) (val-on_b4_b5))
    (:derived (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (or (val-Oon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)))
     (:derived (val-Oon_b1_b2) (or (val-on_b1_b2) (Y-Oon_b1_b2)))
     (:derived (val-Oon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (and (val-Oon_b1_b2) (val-Oon_b2_b3) (val-Oon_b3_b4) (val-Oon_b4_b5)))
     (:derived (val-Oon_b2_b3) (or (val-on_b2_b3) (Y-Oon_b2_b3)))
     (:derived (val-Oon_b3_b4) (or (val-on_b3_b4) (Y-Oon_b3_b4)))
     (:derived (val-Oon_b4_b5) (or (val-on_b4_b5) (Y-Oon_b4_b5)))
     (:derived (val-on_b1_b2) (on b1 b2))
     (:derived (val-on_b2_b3) (on b2 b3))
     (:derived (val-on_b3_b4) (on b3 b4))
     (:derived (val-on_b4_b5) (on b4 b5))
    (:action pick-up
        :parameters (?x - block)
        :precondition (and (clear ?x) (ontable ?x) (emptyhand))
        :effect (and (not (ontable ?x)) (not (clear ?x)) (not (emptyhand)) (holding ?x) (when (not (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)) (not (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5))) (when (not (val-Oon_b1_b2)) (not (Y-Oon_b1_b2))) (when (not (val-Oon_b2_b3)) (not (Y-Oon_b2_b3))) (when (val-Oon_b1_b2) (Y-Oon_b1_b2)) (when (val-Oon_b3_b4) (Y-Oon_b3_b4)) (when (val-Oon_b4_b5) (Y-Oon_b4_b5)) (when (not (val-Oon_b3_b4)) (not (Y-Oon_b3_b4))) (when (not (val-Oon_b4_b5)) (not (Y-Oon_b4_b5))) (when (val-Oon_b2_b3) (Y-Oon_b2_b3)) (when (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)))
    )
     (:action put-down
        :parameters (?x - block)
        :precondition (holding ?x)
        :effect (and (not (holding ?x)) (clear ?x) (emptyhand) (ontable ?x) (when (not (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)) (not (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5))) (when (not (val-Oon_b1_b2)) (not (Y-Oon_b1_b2))) (when (not (val-Oon_b2_b3)) (not (Y-Oon_b2_b3))) (when (val-Oon_b1_b2) (Y-Oon_b1_b2)) (when (val-Oon_b3_b4) (Y-Oon_b3_b4)) (when (val-Oon_b4_b5) (Y-Oon_b4_b5)) (when (not (val-Oon_b3_b4)) (not (Y-Oon_b3_b4))) (when (not (val-Oon_b4_b5)) (not (Y-Oon_b4_b5))) (when (val-Oon_b2_b3) (Y-Oon_b2_b3)) (when (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)))
    )
     (:action stack
        :parameters (?x - block ?y - block)
        :precondition (and (holding ?x) (clear ?y))
        :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (emptyhand) (on ?x ?y) (when (not (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)) (not (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5))) (when (not (val-Oon_b1_b2)) (not (Y-Oon_b1_b2))) (when (not (val-Oon_b2_b3)) (not (Y-Oon_b2_b3))) (when (val-Oon_b1_b2) (Y-Oon_b1_b2)) (when (val-Oon_b3_b4) (Y-Oon_b3_b4)) (when (val-Oon_b4_b5) (Y-Oon_b4_b5)) (when (not (val-Oon_b3_b4)) (not (Y-Oon_b3_b4))) (when (not (val-Oon_b4_b5)) (not (Y-Oon_b4_b5))) (when (val-Oon_b2_b3) (Y-Oon_b2_b3)) (when (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)))
    )
     (:action unstack
        :parameters (?x - block ?y - block)
        :precondition (and (on ?x ?y) (clear ?x) (emptyhand))
        :effect (and (holding ?x) (clear ?y) (not (clear ?x)) (not (emptyhand)) (not (on ?x ?y)) (when (not (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)) (not (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5))) (when (not (val-Oon_b1_b2)) (not (Y-Oon_b1_b2))) (when (not (val-Oon_b2_b3)) (not (Y-Oon_b2_b3))) (when (val-Oon_b1_b2) (Y-Oon_b1_b2)) (when (val-Oon_b3_b4) (Y-Oon_b3_b4)) (when (val-Oon_b4_b5) (Y-Oon_b4_b5)) (when (not (val-Oon_b3_b4)) (not (Y-Oon_b3_b4))) (when (not (val-Oon_b4_b5)) (not (Y-Oon_b4_b5))) (when (val-Oon_b2_b3) (Y-Oon_b2_b3)) (when (val-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5) (Y-OOon_b1_b2-and-Oon_b2_b3-and-Oon_b3_b4-and-Oon_b4_b5)))
    )
)