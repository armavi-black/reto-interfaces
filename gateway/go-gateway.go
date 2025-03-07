package main

import (
	"context"
	"flag"
	"fmt"
	"net/http"

	"github.com/golang/glog"

	"google.golang.org/grpc"

	"github.com/grpc-ecosystem/grpc-gateway/v2/runtime"

	gw "example.com/rest-gateway-demo/protos"
)

var (
	grpcServerEndpoint = flag.String("grpc-server-endpoint", "192.168.3.168:50051", "gRPC svr endpoint")
	gw_port            = "8042"
)

func run() error {
	ctx := context.Background()
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()

	mux := runtime.NewServeMux()
	opts := []grpc.DialOption{grpc.WithInsecure()}
	err := gw.RegisterRobotServiceHandlerFromEndpoint(ctx, mux, *grpcServerEndpoint, opts)
	if err != nil {
		return err
	}

	return http.ListenAndServe(":"+gw_port, mux)
}

func main() {
	flag.Parse()
	defer glog.Flush()
	fmt.Println("Starting Gateway at " + gw_port)
	if err := run(); err != nil {
		glog.Fatal(err)
	}
}
