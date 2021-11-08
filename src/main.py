import machine
from microdot_asyncio import Microdot, Response, redirect, send_file
import network
import json
import uasyncio as asyncio
import director


# station = network.WLAN(network.STA_IF)
# station.active(True)
# station.connect("ComHem<FD40C8>", "*DrowssapATon*")

ssid = 'Tardygrade'
password = ''
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=ssid)
while not ap.active():
    pass
print('network config:', ap.ifconfig())


app = Microdot()

director = director.Director()


@app.route('/favicon.ico', methods=['GET'])
async def favicon(request):
    return send_file('favicon.ico')


@app.route('', methods=['GET', 'POST'])
async def index(request):

    return send_file('webui.html')


@app.route('/move_api', methods=['POST'])
async def move_api(request):

    body = request.body
    move_command = json.loads(body)

    print(move_command)

    d = move_command

    director.recieve_command(d)

    j = json.dumps(d)

    return Response(body=j, headers={'Content-Type': 'application/json'})


async def test():
    i = 0
    while(True):
        print("test "+str(i))
        i = i + 1
        await asyncio.sleep(2)

loop = asyncio.get_event_loop()
t1 = asyncio.create_task(app.start_server(debug=True))
t2 = asyncio.create_task(director.main_loop())
loop.run_until_complete(t1)
loop.run_until_complete(t2)


async def main():
    await app.start_server(debug=True)
    await test()


asyncio.run(main())
