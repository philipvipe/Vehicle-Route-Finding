#!/bin/bash

###############################################################################
# run_demo.bash -- Dana Nau <nau@cs.umd.edu>, Sept 10, 2018
#
# This is a modified version of run_tests.bash. It omits the time test,
# and does just the graphic demo.
#
# You should modify this file to use the sample problems, heuristics, and
# search strategies that you want to use.
###############################################################################


set -f    # disable globbing, because we don't want the name a* to be expanded

###############################################################################
### START OF CUSTOMIZATION OPTIONS


### IMPORTANT!!! Change this to use the pathname for your python program
#
# python=/usr/local/anaconda3/bin/python    # python pathname
python=/Library/Frameworks/Python.framework/Versions/3.6/bin/python3

#### TEST PROBLEMS ###

# Specify the rootname of the file that contains the test problems
#
prob_file=sample_probs

# Below, the first line is a list of all the problems in sample_probs.py, 
# in roughly increasing  order of difficulty. You should modify the second
# line to include just the one(s) you want to use.

# problems=(rect20a rect20b rect20c rect20d rect50 wall8a wall8b lhook16 rhook16a rhook16b spiral16 spiral24 pdes30 pdes30b rectwall8 rectwall16 rectwall32 rectwall32a walls32)

problems=(lhook16)

#### HEURISTIC FUNCTIONS ###

# Here's the rootname of the file that contains the heuristic functions.
# You should replace it with the rootname of YOUR hueuristic function file
#
heur_file=proj1

# Below, the first line is a list of all the heuristic functions in racetrack.py.
# You should modify the second line to include just the one(s) you want to use.

# heuristics=(h_edist h_esdist h_walldist proj1)

heuristics=(proj1 h_walldist)


#### SEARCH STRATEGIES AND OTHER FLAGS ###

# Below, the first line is a list of all the search strategies in heuristics.py. 
# You should modify the second line to include just the one(s) you want to use.

strategies=(bf df uc gbf a*)

strategies=(gbf)

# Specify the amount of verbosity you want fsearch.main to use:
verb=2       

# Use draw=1 to draw the search tree; use draw=0 to draw nothing
draw=0


### End of customization options
#####################################################################


# The following code will iterate over every combination of sample problem, 
# heuristic function, and search strategy in your above lists. For each combination,
# it will first display the results graphically, and then do a time test.
#
for prob in ${problems[*]}
do
  for heur in ${heuristics[*]}
  do
    for strat in ${strategies[*]}
    do
        echo ''
        echo "Graphic display of '$strat, $heur, $prob"

        # strings for preliminary setup, running the program, and printing the result
        setup="import racetrack, $prob_file, $heur_file"
        run_prog="result=racetrack.main($prob_file.$prob, '$strat', $heur_file.$heur, verbose=$verb, draw=$draw, title='$strat, $heur, $prob')"
        print_result="print('\nResult ({} states):\n{}'.format(len(result), result))"
        
        # execute the strings in python
        $python -c "$setup; $run_prog; $print_result"

    done
  done
done
