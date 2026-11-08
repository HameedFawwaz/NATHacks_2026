// Command serve hosts the dashboard's page. With -mavlink it also runs the bridge
// to a vehicle, and the page switches from simulation to live telemetry.
package main

import (
	"context"
	"flag"
	"log"
	"net/http"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/bridge"
)

func main() {
	addr := flag.String("addr", "127.0.0.1:8080", "address to serve the dashboard on")
	dir := flag.String("dir", "web", "directory to serve")
	mavlinkAddr := flag.String("mavlink", "", "UDP address to receive MAVLink on, e.g. 127.0.0.1:14552 (empty = simulation only)")
	publicHost := flag.String("public-host", "", "name or IP the dashboard is reached by from other machines, if any")
	allowRemoteUpload := flag.Bool("allow-remote-upload", false, "allow unauthenticated mission uploads from remote clients (trusted networks only)")
	flag.Parse()

	mux := http.NewServeMux()
	mux.Handle("/", http.FileServer(http.Dir(*dir)))

	if *mavlinkAddr != "" {
		b, err := bridge.Listen(*mavlinkAddr)
		if err != nil {
			log.Fatal(err)
		}
		go func() { log.Fatal(b.Run(context.Background())) }()
		mux.Handle("/api/", b.Handler(bridge.HandlerOptions{
			AllowedHosts:      []string{*addr, *publicHost},
			AllowRemoteUpload: *allowRemoteUpload,
		}))
		log.Printf("listening for MAVLink on udp %s", b.Addr())
	}

	log.Printf("dashboard at http://%s", *addr)
	log.Fatal(http.ListenAndServe(*addr, mux))
}
