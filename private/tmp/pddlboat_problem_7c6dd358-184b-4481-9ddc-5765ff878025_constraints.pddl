(define (problem bw_6)
    (:domain blocks-domain)
    (:objects
        b1 - block
        b2 - block
        b3 - block
        b4 - block
        b5 - block
        b6 - block
    )
    (:init 
        (clear b1)
        (clear b2)
        (clear b3)
        (clear b4)
        (clear b5)
        (clear b6)
        (emptyhand)
        (ontable b1)
        (ontable b2)
        (ontable b3)
        (ontable b4)
        (ontable b5)
        (ontable b6)
    )
    (:goal 
        (and
            (on b2 b3))
    )

    (:constraints
        (and
            (not (holding b2))
            (not (on b2 b3))
    ))
)