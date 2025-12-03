import client
import server

import getpass


if __name__ == "__main__":
    user = getpass.getuser()

    if user == "poisson":
        print("Running server.py as user 'poisson'")
        server.main()
    else:
        print(f"Running client.py as user '{user}'")
        client.main()