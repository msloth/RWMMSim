#encoding: iso-8859-1
# ------------------------------------------------------------------------------
# Author: Marcus Lunden <marcus.lunden@gmail.com>
# 090914
# This file is used for keeping track of the individual nodes that are simulated.
# It's used in the generate-mobility.py RWMM sim; and it is built on some old
# remnant of crappy code I lying around so it is really bad but it works so....
# Would be better to have the nodes in the queue/stack/bastard stored as objects
# and not this mess...
# ------------------------------------------------------------------------------


#states------------
WAITING = 1
MOVING = 2

#------------------------------------------------
#------------------------------------------------
class Node:
    def __init__(self, xx = 0, yy = 0, xgoal = 0, ygoal = 0, xxdot = 0, yydot = 0, stat = 1, wait = 0, ind = 0, trav = 0):
        self.x = xx
        self.y = yy
        self.xg = xgoal
        self.yg = ygoal
        self.xdot = xxdot
        self.ydot = yydot
        self.state = stat
        self.waittime = wait
        self.travel = trav
        self.index = ind
        self.maxspeed = 0
        self.mobility = 0
        self.next = None
        self.prev = None

#------------------------------------------------
class Stack:
    top = None
    last = None
    nonodes = 0
    owner = 0

    def __init__(self, own = 0):
        self.top = None
        self.last = None
        self.nonodes = 0
        self.owner = own
        self.WAITING = 1
        self.MOVING = 2

    def get(self):
        #returns first object in FIFO
        if self.top == None:   #if list is empty, we ret a None
            return None
        tmp = self.top
        self.top = self.top.next
        if self.top != None:
            self.top.prev = None
        tmp.next = None
        tmp.prev = None
        self.nonodes -= 1
        return tmp

    def put(self, x, y, xgoal, ygoal, xxdot, yydot, stat, wai, index):
        #adds new as last in FIFO
        # the index starts from 0, therefore it is right to use self.nonodes before it's ++
        ny_nod = Node(x, y, xgoal, ygoal, xxdot, yydot, stat, wai, index)
        ny_nod.prev = self.last
        if self.nonodes == 0:
            self.top = ny_nod
        else:
            self.last.next = ny_nod
        self.last = ny_nod
        self.nonodes += 1

    def isnotEmpty(self):
        #different semantics so that while(list.isnotEmpty)
        if self.nonodes == 0:
            return False
        else:
            return True

    def logprint(self, time):
        #returns a string suitable for logging purposes.
        if self.nonodes == 0:
            return "Empty list"
        else:
            p = self.top
            st = ""
            for i in range (0, self.nonodes):
                st += str(p.index) + " " + str(time) + " "
                st += str(p.x) + " " + str(p.y) + "\n"
                p = p.next
            return st

    def returntype(self, action):
        # Not yet implemented!...............
        #returns a new list with all the nodes of a specific action
        #syntax:
        # sendlist = Stack()
        # sendlist = list.returntype(ACTION_SEND)
        retlist = Stack(self.owner)
        p = Node()
        p = self.top
        while (p != None):
            if p.action == action:
                #retlist.put(p.timesynch, p.clock, p.action, p.adress)
                print("Not yet implemented!")
            p = p.next
        return retlist


    def __str__(self):
        #for printing the FIFO
        if self.top == None:
            st = "Queue is empty"
        else:
            st = "--- Queue: ---\n"
            st += "No nodes: " + str(self.nonodes) + "\n"
            p = self.top
            for i in range (0, self.nonodes):
                st += ("\nIndex: " + str(p.index) + "\n")
                st += "At:" + str(p.x) + "[m] " + str(p.y) + "[m]\n"
                if p.state == WAITING:
                    st += "WAITING time left: \n" + str(p.waittime) + " [s]\n"
                else:
                    st += "MOVING to:\n"
                    st += str(p.xg) + "[m] " + str(p.yg) + "[m]\n"
                    st += "Speed:" + str(p.xdot) + "[m/s] " + str(p.ydot) + "[m/s]\n"

                p = p.next
            st += "End of list. \n--------------------------\n"
        return st
#------------------------------------------------

if __name__=="__main__":
  print("Part of RWMMsim.")
  print("this is the node list queue file; use the 'generate-mobility.py' instead!")
#	main()

