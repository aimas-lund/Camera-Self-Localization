import time


class log():
    def __init__(self, name, no_header=False, log_time=True, split_string=' '):
        self.name = './log/' + name + '.log'
        self.no_header = no_header
        self.log_time = log_time
        
        self.headers = []
        self.data = []

        # Flag used to see if data should be appended or overwritten
        self.data_written = False
        
        self.split_string = split_string
        
        self.last_data = []
    
    
    def set_headers(self, *args):
        if self.no_header:
            return
        # Reset the headers
        self.headers = []
        # If the first input is a list, then use that
        if len(args[0]) > 1:
            for arg in args[0]:
                self.headers.append(arg)
        else:
            # Fill the headers
            for arg in args:
                self.headers.append(arg)

    
    def log(self, *args, only_new_data=True):
        if len(self.headers) == 0 and not self.no_header:
            print("No headers set, using default 1,2,...")
            tmp = [i for i in range(len(args))]
            self.set_headers(tmp)


        # Fill up the data
        tmp = []
        for arg in args:
            tmp.append(arg)
            
        # Log the time
        if self.log_time:
            tmp.append(time.time())
            # Switch the time to be the first element
            tmp = [tmp[-1]] + tmp[:-1]
        
        # Only add new data
        if only_new_data and tmp == self.last_data:
            return
        
        # Add this measurement to the data
        self.data.append(tmp)
        
        self.last_data = tmp
    
    # Only do in the end of the program
    def write(self):
        if not self.data_written:
            open_symbol = 'w'
        else:
            open_symbol = 'a'
        
        f = open(self.name, open_symbol)
        # Write header only the first time
        if not self.data_written and not self.no_header:
            f.write(self.list_to_string(self.headers))
        # Write the data
        for info in self.data:
            f.write(self.list_to_string(info))
        f.close()
        
        self.data_written = True
        # Erase the data, so it is not written again
        self.data = []
        
    
    def list_to_string(self, _list):
        string = ''
        for data in _list:
            string += str(data) + self.split_string
        
        # Remove fencepost error
        string = string[:-len(self.split_string)]
        # Add new line
        string += '\n'
        return string
            