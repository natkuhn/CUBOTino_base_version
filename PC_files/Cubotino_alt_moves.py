# Cubotino_alt_moves.py
# Nat Kuhn 5/23/2024
# Thanks, Andrea!

# The orientation of the cube is specified by two letters
# The first letter is which face is on the bottom
# The second letter is which face faces forward
# The default starting position for the cube is "DF"

MOVES = { "DF" : { "f": "FU", "p": "DL", "n": "DR" },
          "DR" : { "f": "RU", "p": "DF", "n": "DB" },
          "DB" : { "f": "BU", "p": "DR", "n": "DL" },
          "DL" : { "f": "LU", "p": "DB", "n": "DF" },
           
          "FU" : { "f": "UB", "p": "FL", "n": "FR" },
          "FR" : { "f": "RB", "p": "FU", "n": "FD" },
          "FD" : { "f": "DB", "p": "FR", "n": "FL" },
          "FL" : { "f": "LB", "p": "FD", "n": "FU" },

          "UB" : { "f": "BD", "p": "UL", "n": "UR" },
          "UR" : { "f": "RD", "p": "UB", "n": "UF" },
          "UF" : { "f": "FD", "p": "UR", "n": "UL" },
          "UL" : { "f": "LD", "p": "UF", "n": "UB" },
         
          "BD" : { "f": "DF", "p": "BL", "n": "BR" },
          "BR" : { "f": "RF", "p": "BD", "n": "BU" },
          "BU" : { "f": "UF", "p": "BR", "n": "BL" },
          "BL" : { "f": "LF", "p": "BU", "n": "BD" },
         
          "RF" : { "f": "FL", "p": "RD", "n": "RU" },
          "RU" : { "f": "UL", "p": "RF", "n": "RB" },
          "RB" : { "f": "BL", "p": "RU", "n": "RD" },
          "RD" : { "f": "DL", "p": "RB", "n": "RF" },
         
          "LF" : { "f": "FR", "p": "LU", "n": "LD" },
          "LD" : { "f": "DR", "p": "LF", "n": "LB" },
          "LB" : { "f": "BR", "p": "LD", "n": "LU" },
          "LU" : { "f": "UR", "p": "LB", "n": "LF" }
        }


# The state of the robot is specified by cube orientation, bottom servo,
# and top servo position

# orientation is the two-letter string, as described above.
 
# top_servo can be "c" (closed), "o" (open), or "f" (flip)

# bottom_servo can by -1 (CCW), 0 (HOME), or 1 (CW)

class RobotState:
    def __init__(self,orientation,bottom_servo, top_servo):
        self.orientation = orientation
        self.bottom_servo = bottom_servo
        self.top_servo = top_servo
    def copy(self):
        return RobotState(self.orientation,self.bottom_servo,self.top_servo)
    def __str__(self):
        return f"orientation = {self.orientation}, bottom_servo = {self.bottom_servo}, top_servo = {self.top_servo}"

# each move for the robot is an instance of RobotMove
# it's a little different from the original system
# Rotate in this system can be the original R (if the top is closed)
# or S (Spin) if the top is open

class RobotMove:
    def updateState(self,initial_state):
        pass
    def __str__(self):
        return f'Robot command: {self.command_string}'

class Flip(RobotMove):
    def updateState(self,initial_state):
        self.command_string = "fo" # move up to flip position, and back to open
        state = initial_state.copy()
        state.orientation = MOVES[state.orientation]['f']
        return state

class Close(RobotMove):
    def updateState(self,initial_state):
        self.command_string = "c"
        state = initial_state.copy()
        state.top_servo = "c"
        return state

class Open(RobotMove):
    def updateState(self,initial_state):
        self.command_string = "o"
        state = initial_state.copy()
        state.top_servo = "o"
        return state

BOTTOM_SERVO_COMMAND = ('l', 'h', 'r')

class Rotate(RobotMove):
    def __init__(self,turn):
        self.turn = turn
    def updateState(self,initial_state):
        if self.turn == 1:
            if initial_state.bottom_servo == 1:
                raise Exception("Cannot move +1 from CW position")
            state = initial_state.copy()
            state.bottom_servo += 1
            self.command_string = BOTTOM_SERVO_COMMAND[state.bottom_servo + 1]
            if state.top_servo == "o":
                state.orientation = MOVES[state.orientation]['p']
            return state
        if self.turn == -1:
            if initial_state.bottom_servo == -1:
                raise Exception("Cannot move -1 from CCW position")
            state = initial_state.copy()
            state.bottom_servo -= 1
            self.command_string = BOTTOM_SERVO_COMMAND[state.bottom_servo + 1]
            if state.top_servo == "o":
                state.orientation = MOVES[state.orientation]['n']
            return state
        if self.turn == 2:
            if initial_state.bottom_servo == 0:
                raise Exception("Cannot move +/-2 from HOME position")
            state = initial_state.copy()
            state.bottom_servo = -state.bottom_servo
            self.command_string = BOTTOM_SERVO_COMMAND[state.bottom_servo + 1]
            if state.top_servo == "o":
                state.orientation = MOVES[state.orientation]['n']
                state.orientation = MOVES[state.orientation]['n'] # move twice, in either direction
            return state
        raise Exception("Invalid number of turns in Rotate")
    
# each move from the Kociemba solver becomes an instance of SolverMove
    
class SolverMove:
    def __init__(self,solver_move):
        self.face = solver_move[0]
        self.twist = int(solver_move[1])
        if self.twist == 3:
            self.twist = -1
    
    def __str__(self):
        return f"{self.face}{self.twist}"
        
    def robotMoves(self,initial_state):
        self.robot_moves = []
        state = initial_state
        
        def moveToBottom(direction):
            nonlocal state, self
            if state.bottom_servo != direction: # we can do that move
                addMove(Rotate(direction))
                addMove(Flip())
                return state
            else: # can't turn that way, bring desired face to back
                addMove(Rotate(-direction))
                for i in range(3):
                    addMove(Flip())
                return state

        def addMove(move):
            nonlocal state, self
            state = move.updateState(state)
            self.robot_moves.append(move)
            return
        
        # first, we need to move "face" to the bottom
        iso = state.orientation
        if iso[0] == self.face:   # already on the bottom
            return initial_state
        here = MOVES[iso]
        if here['p'][1] == self.face: # can bring to front with +1 twist
            moveToBottom(1)
        elif here['n'][1] == self.face: # can bring to front with -1 twist
            moveToBottom(-1)
        else: # we can get there with flips
            for i in range(3):
                addMove(Flip())
                if state.orientation[0] == self.face:
                    break

        if state.orientation[0] != self.face: # wrong place after 3 flips
            # or some other error
                raise Exception("Didn't get to the face we need")

        # OK, the correct face is on the bottom
        if self.twist == 2:
            if state.bottom_servo == 0:
                addMove(Rotate(-1)) # could be +1, there is room for optimization here
        else: # twist is +/-1
            if state.bottom_servo == self.twist:
                addMove(Rotate(-self.twist)) # need to move to HOME position

        # finally, we can do the rotation
        addMove(Close())
        addMove(Rotate(self.twist))
        addMove(Open())
        return state
    
def robot_moves(solution):
    solver_list = []
    while solution != '':
        s = solution[0]
        if ' \n\r'.find(s) >=0:
            solution = solution[1:]
            continue
        solver_list.append( SolverMove(solution[0:2]) )
        solution = solution[2:]

    state = RobotState("DF",0,"o")
    robot_list = []

    for m in solver_list:
        # print(f"Initial state: {str(state)}, SolverMove: {str(m)}")
        state = m.robotMoves(state)
        robot_list.append( ''.join([x.command_string for x in m.robot_moves]) )
    
    return ' '.join(robot_list)
