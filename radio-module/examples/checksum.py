def send_command(command, add_checksum=True):
        """Send a command string to the GPS.  If add_checksum is True (the
        default) a NMEA checksum will automatically be computed and added.
        Note you should NOT add the leading $ and trailing * to the command
        as they will automatically be added!
        """
        # self.write(b"$")
        # self.write(command)
        if add_checksum:
            checksum = 0
            for char in command:
                #print(ord(char))
                checksum ^= ord(char)
            print(checksum)
            # self.write(b"*")
            print(bytes("{:02x}".format(checksum).upper(), "ascii"))
        print(b"\r\n")


send_command("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
print("new command")
send_command('PMTK220,1000')