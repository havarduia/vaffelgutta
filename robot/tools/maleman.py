
class MaleMan:
    def __init__(self) -> None:
        self.recievers = dict() 

    def register(self, name: str, obj):
        self.recievers.update({name : obj})

    def send_male(self, reciever: str, operation: str, msg: any):
        try:
            self.recievers[reciever].rxmsg(operation, msg)
        except AttributeError:
            print(f"There is no rxmsg function in the {reciever} object!")
    # Define aliases
    txmsg = send_male 
    send_message = send_male 
    sendmsg = send_male 


