from main.waffle_states.waffle_states import State, CurrentState
from main.state_transitions.open_iron import open_iron
state = State()
bot = 2
def change_state_example():
    current_state_obj = CurrentState()
    open_iron(state, bot)
    while True:
        print(current_state_obj.get())

if __name__ == "__main__":
    change_state_example()
