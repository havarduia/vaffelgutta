from argparse import ArgumentParser
def read_input_args():
    parser = ArgumentParser(description="Runs a wafflebot")
    parser.add_argument("-r", required=False, type=int, default=0, help="1 to use real robot, 0 to simulate")
    args = parser.parse_args() 
    return bool(args.r)