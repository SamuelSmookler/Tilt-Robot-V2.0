class CommandError(Exception):
    pass
            




class Link:
    def __init__(self, transport):
        self._transport = transport
        self._tag = 1
        self._events = []
        
    def next_tag(self):
        tag = self._tag
        self._tag += 1
        return tag
    
    def command(self, text):
        tag = self.next_tag()
        self._transport.write_line(f"#{tag} {text}")
        prefix = f"#{tag} "
        while True:
            line = self._transport.read_line()
            if line is None:
                raise RuntimeError(f"no reply for tag {tag}")
            if line.startswith("EVT "):
                self._events.append(line)
                continue
            if line.startswith(prefix):
                # return line[len(prefix):]
                payload = line[len(prefix):]
                
                
                if payload.startswith("ERR"):
                    raise CommandError(payload)
                else:
                    return payload
                
        
                
            
    
        
