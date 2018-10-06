import _crc8cy

# Mostly copy pasted from Nicco Kunzmanns crc8 code.

class crc8(object):

    digest_size = 1
    block_size = 1
    
    def __init__(self, initial_string=b''):
        """Create a new crc8 hash instance."""
        self._sum = 0x00
        self._update(initial_string)

    def update(self, bytes_):
        """Update the hash object with the string arg.

        Repeated calls are equivalent to a single call with the concatenation
        of all the arguments: m.update(a); m.update(b) is equivalent
        to m.update(a+b).
        """
        self._update(bytes_)

    def digest(self):
        """Return the digest of the bytes passed to the update() method so far.

        This is a string of digest_size bytes which may contain non-ASCII
        characters, including null bytes.
        """
        return chr(self._sum)

    def hexdigest(self):
        """Return digest() as hexadecimal string.

        Like digest() except the digest is returned as a string of double
        length, containing only hexadecimal digits. This may be used to
        exchange the value safely in email or other non-binary environments.
        """
        return hex(ord(self._sum))[2:].zfill(2)

    def copy(self):
        """Return a copy ("clone") of the hash object.
        
        This can be used to efficiently compute the digests of strings that
        share a common initial substring.
        """
        crc = crc8()
        crc._sum = self._sum
        return crc

    def _update(self, bytes_):
        self._sum = _crc8cy.update_crc8(bytes_, self._sum)
