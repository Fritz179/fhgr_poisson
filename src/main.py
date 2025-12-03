
import getpass

if __name__ == "__main__":
    user = getpass.getuser()

    if user == "poisson":
        print("Running server.py as user 'poisson'")
        
        import server
        server.main()
    else:
        print(f"Running client.py as user '{user}'")

        import client
        client.main()