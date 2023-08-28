const dgram = require('dgram');

const DST_PORT = 10001;
const DST_HOST = '192.168.2.102';

const SRC_PORT = 10001;
const SRC_HOST = '192.168.2.109';

const socket = dgram.createSocket('udp4');

var count = 0;

setInterval(() => {
    count++;
    const data = Buffer.from(String(count));
    socket.send(data, 0, data.length, DST_PORT, DST_HOST, (err, bytes) => {
        if (err) throw err;
    });
}, 500);


socket.on('message', (message, remote) => {
    console.log(remote.address + ':' + remote.port +' - ' + message);
});

socket.bind(SRC_PORT, SRC_HOST);

