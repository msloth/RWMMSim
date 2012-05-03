#encoding: iso-8859-1
#
# Author: Marcus Lunden <marcus.lunden@gmail.com>
# Generates a file with data for a simulated WSN in motion.
# The parameters are either stated in a textfile or prompted
#
# Changelog:
#   0.02: first versions.
#   0.03: added floats for allowing fractions of meters and seconds on parameters
#   0.04: added writing to logfile for plotting movement of individual node
#   0.05: added simple statistics: min&max travel, speeds, and save logg stats to file
#   0.06: bugfixes and more statistics (2009-oct-09)
#   0.07: removed the 'from import *', they are bad due to overloading and namespace confusion
#   0.08: added functionality to generate a new config file if last one is borked (2012-may-01)
#   0.09: removed plotting of 3 first nodes, it was a temporary thing anyway. Thanks Teo Kar Hoey!
#   0.10: protected against non-valid input in conf file, disregards any #comments from .txt file no matter where they come
#   
#   

# Planned:
#   * mess of mixed global variables and passed args to functions
#   * statistical metric on mobility: in what percentile is each node? Most mobile? Bottom 10%?
#   * 
#   .......
# ------------------------------------------------------------------------------

import WSNnodelist
import random
import math
import os
import Numeric
import genConfFile

# global variables --------------
# program version
version = "0.10"
date = "2012-may-02"
model = "Random Waypoint model"     # for now, there is only one model

# change these to reflect on your true file names
# Note, it spits out a few files that I needed at the time for plotting subsets
# of the generated data, and some statistics.
config_file = "generate-mobility.txt"
output_file = "positions.dat"    # the data file for cooja compatibility
stat_filename = "stat.txt"

# used in simulation
timesteps = 0   # time ticks/second (Hz)
resolution = 0  # time step (s)
starttime = 0   # time when logging starts (s) (0 + disregardtime)
realtime = 0    # calculated real time (s) since log starts (because the sim loop uses ticks)
globalticks = 0 # total no of ticks since start of sim (ticks)
DIST_RES = 0    # resolution in distance, for knowing when a node is in vicinity of its goal

# states
WAITING = 0     # state == waiting
MOVING = 1      # state == moving
OOR = 0     # radio has no connectivity
IIR = 1     # radio is in radio interference range
INR = 2     # radio is in full connectivity range

# these are parameters read from input or config file
noofnodes = 0
maxtime = 0
resolution = 0
minpause = 0
maxpause = 0
maxx = 0
maxy = 0
minspeed = 0
maxspeed = 0
disregardtime = 0

#---------------------------------------------------------------------------
# Updates the logfile
def update_log(list, file, time):
    # saves the state to file. Time is given by variable time.
    # format:
    #  index time x y
    ss = list.logprint(time)
    file.write(ss)

#---------------------------------------------------------------------------
# Calculates the new position based on present location and speed
# also adds to the total amount of travel by calling distance function
def new_pos(pp):
    global globalticks, starttime
    prevx = pp.x
    prevy = pp.y
    if pp.x < pp.xg:
        pp.x = pp.x + pp.xdot * resolution
    else:
        pp.x = pp.x - pp.xdot * resolution
    if pp.y < pp.yg:
        pp.y = pp.y + pp.ydot * resolution
    else:
        pp.y = pp.y - pp.ydot * resolution
    if globalticks >= starttime:
        tr = distance(pp.x, pp.y, prevx, prevy)
        pp.travel += tr
    return pp

#---------------------------------------------------------------------------
# This function does the actual movement of the nodes, using the random
#    waypoint mobility model.
def update_world(list, timeres):
    global rangelist_moment
    global rangelist_global

    p = list.top
    #----------------------------------------------
    # update states and parameters (speed, position etc)
    for i in range(0, list.nonodes):
        if p.state == MOVING:
            p = new_pos(p)
            if distance(p.x, p.y, p.xg, p.yg) <= DIST_RES:          # reached the goal
                p.state = WAITING
                p.waittime = generate_sleeporspeed(minpause, maxpause)

        if p.state == WAITING:
            if p.waittime <= timeres:                               # now done waiting
                sp = generate_sleeporspeed(minspeed, maxspeed)      # generate speed
                if sp > p.maxspeed:
                    p.maxspeed = sp
                p.state = MOVING                                    # change to moving
                p.xg, p.yg = generate_waypoint(maxx, maxy)          # generate new goal
                p.xdot, p.ydot = calculate_speedvector(p.x, p.y, p.xg, p.yg, sp)  # calc new speedvector
            else:
                p.waittime -= timeres            # little less waiting left
        p = p.next
    # end for--------------------------------------


#---------------------------------------------------------------------------
# calculates the distance between two points in 2D-space w Pythagoras
def distance(x, y, xg, yg):
    dx = x-xg
    dy = y-yg
    d = math.sqrt((dy**2)+(dx**2))
    return d

#---------------------------------------------------------------------------
# calculates the distance between two nodes in 2D-space w Pythagoras
# difference between this and distance() is that this takes two _nodes_ and not
# two points.
def distancenodes(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    d = math.sqrt((dy**2)+(dx**2))
    return d

#---------------------------------------------------------------------------
# returns the coordinates for a randomly chosen waypoint
def generate_waypoint(maxxx, maxxy):
    x = random.uniform(0, maxxx)
    y = random.uniform(0, maxxy)
    return x, y

#---------------------------------------------------------------------------
# returns a random value w uniform distribution in the interval
def generate_sleeporspeed(mini, maxi):
    s = random.uniform(mini, maxi)
    return s


#---------------------------------------------------------------------------
# calculates the x and y components from the speed vector
def calculate_speedvector(x, y, xg, yg, s):
    dx = abs(x-xg)
    dy = abs(y-yg)
    temp = ((dx**2)*(dy**2))/((dy**4)+(dx**2)*(dy**2))
    xdot = s * math.sqrt(temp)
    ydot = s * math.sqrt(1-temp) #temp always >=0
    return xdot, ydot

#---------------------------------------------------------------------------
# calculates the x and y components from the speed vector
def get_next_arg(fil):
    s = fil.readline()
    while s[0] == '#':
      s = fil.readline()
    return s

#---------------------------------------------------------------------------
# Main function
def main():
    # here are all the global variables again...
    global timesteps
    global resolution
    global starttime
    global realtime
    global globalticks
    global noofnodes
    global maxtime
    global resolution
    global minpause
    global maxpause
    global maxx
    global maxy
    global minspeed
    global maxspeed
    global disregardtime
    global interfere
    global recepetion
    global DIST_RES

    #print some hellos~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    print("\n\n\n--------------------------------------------------")
    print("Random waypoint mobility model generator for WSN nodes")
    print("Author Marcus Lundén, SICS")
    print("Version: " + version + ", " + date)
    print("Almost no error checking whatsoever is performed, don't do anything wrong ;)")
    print("\n--------------------------------------------------")
    print("Init...")

    # aquire parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    try:
        conf = open(config_file, 'r')
        noofnodes = int(get_next_arg(conf))
        maxtime = float(get_next_arg(conf))
        resolution = float(get_next_arg(conf))
        minpause = float(get_next_arg(conf))
        maxpause = float(get_next_arg(conf))
        maxx = float(get_next_arg(conf))
        maxy = float(get_next_arg(conf))
        minspeed = float(get_next_arg(conf))
        maxspeed = float(get_next_arg(conf))
        disregardtime = float(get_next_arg(conf))
        interfere = float(get_next_arg(conf))
        reception = float(get_next_arg(conf))
        conf.close()
        print("Used configuration file: " + config_file + "\n")
    except:
        print("Did not find configuration file, or contains non-valid conf: " + config_file)
        print("Will autogenerate a template and save as: " + config_file)
        genConfFile.new(config_file)
        print("\nNow please state parameters manually.")
        noofnodes = int(input("No of nodes: "))
        maxtime = float(input("Time [s]: "))
        resolution = float(input("Resolution [s] (eg 0.01 gives 1/100th s resolution): "))
        minpause = float(input("Minimum pause time at waypoint [s]: "))
        maxpause = float(input("Maximum pause time at waypoint [s]: "))
        maxx = float(input("Max X-size [m]: "))
        maxy = float(input("Max Y-size [m]: "))
        minspeed = float(input("Minimum travel speed [m/s]: "))
        maxspeed = float(input("Maximum travel speed [m/s]: "))
        disregardtime = float(input("How long time to disregard from start [s]: "))
        interfere = float(input("Radio interference range [m]: "))
        reception = float(input("Radio reception range [m]: "))
    
    #keep on going until max time has been reached, but disregard the start.
    maxtime = maxtime + disregardtime

    #minor error checking----------------------------
    if minspeed > maxspeed:
        # user switched places on min and max
        temp = minspeed
        minspeed = maxspeed
        maxspeed = temp

    if minpause > maxpause:
        # user switched places on min and max
        temp = minpause
        minpause = maxpause
        maxpause = temp

    #generate waypoints as linked list------------------------
    # init linked list
    nodelist = WSNnodelist.Stack()

    # create the actual nodes
    for i in range(0, noofnodes):
        # generate start points, goals and calculate speed vectors
        # also randomize whether they are moving or waiting to begin with
        # with the same probability for both cases.
        ss = generate_sleeporspeed(minspeed, maxspeed)          # speed
        xx, yy = generate_waypoint(maxx, maxy)                  # position
        xgg, ygg = generate_waypoint(maxx, maxy)                # goal
        xdo, ydo = calculate_speedvector(xx, yy, xgg, ygg, ss)  # speed
        if random.random() <= 0.5:                                     # state
            state = WAITING
            time = generate_sleeporspeed(minpause, maxpause)    # sleeptime
        else:
            state = MOVING
            time = 0
        nodelist.put(xx, yy, xgg, ygg, xdo, ydo, state, time, i)   # add node to list

    #print(nodelist)    # prints the list in a convenient way

    #open the file for writing------------------------------
    fil = open(output_file, 'w')

    #write intro section to logfile-------------------------
    fil.write("# position.dat\n")
    fil.write("#\n")
    fil.write("# Position data for simulation of WSN nodes\n")
    fil.write("# Automatically generated with Python\n")
    fil.write("# Version: " + version + ", " + date)
    fil.write("\n# Author: Marcus Lundén, SICS\n")
    fil.write("# email: marcus.lunden@gmail.com\n")
    fil.write("#\n")
    fil.write("# Model used: " + model + "\n")
    fil.write("# Number of nodes: " + str(noofnodes) + "\n")
    fil.write("# Time [s]: " + str(maxtime-disregardtime) + " seconds\n")
    fil.write("# Min speed [m/s]: " + str(minspeed) + "\n")
    fil.write("# Max speed [m/s]: " + str(maxspeed) + "\n")
    fil.write("# Min pause time [s]: " + str(minpause) + "\n")
    fil.write("# Max pause time [s]: " + str(maxpause) + "\n")
    fil.write("# Currently disregarding the first " + str(disregardtime) + " seconds\n")
    fil.write("# Resolution [s]: " + str(resolution) + "\n")
    fil.write("# Maximum X-size [m]: " + str(maxx) + "\n")
    fil.write("# Maximum Y-size [m]: " + str(maxy) + "\n")
    fil.write("# Note that nodes index here start at zero due to Cooja input format.\n")
    fil.write("#    However, the nodes are indexed >=1 in stats file.\n")
    fil.write("#\n#==========================================\n")

    # calculate variables used in time loop
    timesteps = int(maxtime * (1/resolution))
    starttime = disregardtime * int(1/resolution)

    # calculate the maximum amount of travel one node can travel when at max speed
    #   in one interval (dist = speed * time)
    DIST_RES = maxspeed * resolution

    # setup arrays for keeping track of connectivity
    rangelist_now = Numeric.zeros([noofnodes, noofnodes], Numeric.Int)
    rangelist_pre = Numeric.zeros([noofnodes, noofnodes], Numeric.Int)
    logst = ""              # string used for saving connection logs
    rangestr = ""           # keep all rangelogs in this string, to be printed/ andor written to file later
    timenow = (globalticks * resolution) - disregardtime

    # -------------------------------------------------------------------------
    # main loop that moves the nodes and updates states -----------------------
    for globalticks in range(0, timesteps):
        # timesteps == ticks. Timestep in [s] = timesteps * resolution
        # global time since sim start == (globalticks * resolution) - disregardtime

        #update the positions and states---------------------------
        update_world(nodelist, resolution)

        # update connectivity array--------------------------------
        # iterate through the nodes and check distances, filling in the range
        # we go along. It becomes an upper triangular matrix (w the diagonal elements unused)
        # Keep two versions of this matrix, one that is one step behind so that we
        # can keep track of changes by checking if a value has changed since the timestep before
        p = nodelist.top
        timenow = (globalticks * resolution) - disregardtime
        for i in range (0, noofnodes-1):
            # for every node, check dist to nodes w higher index
            tempstr = ""
            q = p.next
            for t in range(i+1, noofnodes):
                if q != None:
                    #check ranges
                    dd = distance(p.x, p.y, q.x, q.y)

                    if dd <= reception:
                        rangelist_now[i][t] = 2   #in range
                        if rangelist_pre[i][t] != 2:
                            tempstr = str(timenow) + "\t\t" + str(p.index+1) + "\t" + str(q.index+1)+"   IN range\n"
                            rangelist_pre[i][t] = 2
                        # commented out interference range...
                        """
                            elif dd <= interfere:
                                rangelist_now[i][t] = 1   #interference range
                                if rangelist_pre[i][t] != 1:
                                    # XXX: changed line so that interference range is denoted the same as
                                    #      out of range because that only important that they are not in range!
                                    tempstr = str(timenow) + "\t\t" + str(p.index+1) + "\t" + str(q.index+1)+"   OUT OF range\n"
                                    #tempstr = str(timenow) + "\t\t" + str(p.index+1) + "\t" + str(q.index+1)+"   INTERFERE range\n"
                                    rangelist_pre[i][t] = 1
                        """
                    elif dd>reception:
                        rangelist_now[i][t] = 0   #out of range
                        if rangelist_pre[i][t] != 0:
                            # Changed so that Out of range gives rise to no message at all, see not for interference.
                            tempstr = str(timenow) + "\t\t" + str(p.index+1) + "\t" + str(q.index+1)+"   OUT OF range\n"
                            rangelist_pre[i][t] = 0
                    if (globalticks * resolution) > disregardtime:
                        rangestr += tempstr
                        tempstr = ""
                    q = q.next
            p = p.next
        if timenow==0:
            # print the connectivity array
            rl = str(rangelist_now)
            rangestr += "\n\n Connectivity matrix at simulation start.\n"
            rangestr += "Node with lowest index is in the top left corner.\n"
            rangestr += "Legend: %d means IN range, %d means Interference range, %d means out of range.\n"%(INR, IIR, OOR)
            rangestr += "Eg. [1][3]==%d means that nodes 1 and 3 are in range of eachother.\n\n"%(INR)
            rangestr += "#NONODES: %i\n"%(noofnodes)
            rangestr += "\n#CONNSTART\n" + rl +"\n#CONNEND\n"
            rangestr += "\n\n#STATSTART\n"
            #print rl


        #write generated data to file---------------------------
        # only writes the data for three nodes so that they can be plotted
        # as examples when doing figures
        if globalticks >= starttime:
            realtime = (globalticks - starttime) * resolution
            update_log(nodelist, fil, realtime)

    # log the final connectivity matrix
    rl = str(rangelist_now)
    rangestr += "#STATEND\n\n Connectivity matrix at simulation end.\n"
    rangestr += rl + "\n\n"
    print rl

    # -------------------------------------------------------------------------


    #clean up-----------------------------------------------
    fil.close()

    # print some post-generation messages.
    st = "\nGeneration done, generated results are found in file: " + output_file
    st += "\nStatistical results are found in file: " + stat_filename
    st += "\nTotal number of nodes: " + str(noofnodes)
    st += "\nSimulation time: " + str(maxtime - disregardtime) + "[s] and " + str(disregardtime) + "[s] are disregarded"
    st += "\nArea: " + str(maxx) + "*" + str(maxy)+" [m^2]\n\n"
    print(st)

    # accumulate stats and write stats -------------------------------------------------------

    # check and print statistics for which node has travelled the most
    statfd = open(stat_filename, 'w')
    maxm = 0
    ind = 0
    minm = maxtime * maxspeed
    minind = 0
    tsum = 0
    t = nodelist.top

    rangestr += "  \n\nNode\t\t Avg speed [m/s]\t Max speed[m/s]\t\t Travel[m]\n"
    rangestr += "====================================================================\n"
    for k in range(0, nodelist.nonodes):

        # individual stats (speed, travel)
        rangestr += "\t" + str(t.index+1) + "\t\t %.2f"%(t.travel/maxtime)
        rangestr += "\t\t\t\t %(spd).2f \t\t\t\t %(tra).2f \n"%{'spd':t.maxspeed, 'tra':t.travel}

        # global stats
        if t.travel > maxm:
            maxm = t.travel
            ind = t.index + 1
        if t.travel < minm:
            minm = t.travel
            minind = t.index +1
        tsum += t.travel
        t = t.next

    rangestr += "\n\nMaximum travel by node: "+str(ind)+"\nDistance: %.2f"%maxm
    rangestr += " [m]. AvgSpeed: %.2f"%(maxm/maxtime) + " [m/s].\n"

    rangestr += "Minimum travel by node: "+str(minind)+"\nDistance: %.2f"%minm
    rangestr += " [m]. AvgSpeed: %.2f"%(minm/maxtime) + " [m/s].\n"

    statfd.write("# Position data for simulation of WSN nodes\n")
    statfd.write("# Automatically generated\n")
    statfd.write("# Version: " + version + ", " + date)
    statfd.write("\n# Author: Marcus Lundén, SICS\n")
    statfd.write("# email: marcus.lunden@gmail.com\n")
    statfd.write("#\n")
    statfd.write("# Model used: " + model + "\n")
    statfd.write("# Number of nodes: " + str(noofnodes) + "\n")
    statfd.write("# Time [s]: " + str(maxtime-disregardtime) + " seconds\n")
    statfd.write("# Min speed [m/s]: " + str(minspeed) + "\n")
    statfd.write("# Max speed [m/s]: " + str(maxspeed) + "\n")
    statfd.write("# Min pause time [s]: " + str(minpause) + "\n")
    statfd.write("# Max pause time [s]: " + str(maxpause) + "\n")
    statfd.write("# Currently disregarding the first " + str(disregardtime) + " seconds\n")
    statfd.write("# Resolution [s]: " + str(resolution) + "\n")
    statfd.write("# Maximum X-size [m]: " + str(maxx) + "\n")
    statfd.write("# Maximum Y-size [m]: " + str(maxy) + "\n")
    statfd.write("#\n#==========================================\n")

    print(rangestr)
    statfd.write(rangestr)
    statfd.close()



#-----------------------------------------------------------------------------

if __name__=="__main__":
	main()
