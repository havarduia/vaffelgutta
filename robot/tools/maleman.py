
class MaleMan:
    def __init__(self) -> None:
        self.recievers = dict() 

    def register(self, name: str, obj):
        self.recievers.update({name : obj})

    def send_male(self, reciever: str, msg: str):
        try:
            self.recievers[reciever].rxmsg(msg)
        except AttributeError:
            print("There is no rxmsg function in this object!")
    txmsg = send_male 
    send_message = send_male 
    sendmsg = send_male 


