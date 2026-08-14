import 'package:examples/files/area_costs.dart';
import 'package:examples/files/area_filters.dart';
import 'package:examples/files/chunky_tri_mesh.dart';
import 'package:examples/files/solo_navmesh.dart';
import 'package:examples/src/demo.dart';
import 'package:examples/src/files_json.dart';
import 'package:flutter/material.dart';
import 'package:css/css.dart';
import 'src/plugins/plugin.dart';

void main() {
  setPathUrlStrategy();
  runApp(const MyApp());
}
class MyApp extends StatefulWidget{
  const MyApp({super.key,}) ;
  @override
  MyAppState createState() => MyAppState();
}

class MyAppState extends State<MyApp> {
  final GlobalKey<NavigatorState> _navKey = GlobalKey<NavigatorState>();
  String onPage = '';
  double pageLocation = 0;

  @override
  void initState() {
    super.initState();
  }

  void callback(String page, [double? location]){
    onPage = page;
    if(location != null){
      pageLocation = location;
    }
    WidgetsBinding.instance.addPostFrameCallback((timeStamp) { 
      _navKey.currentState!.popAndPushNamed('/$page');
      setState(() {});
    });
  }

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    widthInifity = MediaQuery.of(context).size.width;
    return SafeArea(
      child: MaterialApp(
        debugShowCheckedModeBanner: false,
        title: 'NAVCAT',
        theme: CSS.darkTheme,
        home: Scaffold(
          appBar: onPage != ''? PreferredSize(
            preferredSize: Size(widthInifity,65),
            child:AppBar(callback: callback,page: onPage,)
          ):null,
          body: MaterialApp(
            debugShowCheckedModeBanner: false,
            title: 'NAVCAT',
            theme: CSS.darkTheme,
            navigatorKey: _navKey,
            routes: {
              '/':(BuildContext context) {
                return Examples(
                  callback: callback,
                  prevLocation: pageLocation,
                );
              },
              '/area_costs':(BuildContext context) {
                return const AreaCosts();
              },
              '/area_filters':(BuildContext context) {
                return const AreaFilters();
              },
              '/chunky_tri_mesh':(BuildContext context) {
                return const ChunkyTriMeshEx();
              },
              '/crowd_simulation':(BuildContext context) {
                return const Demo();
              },
              '/custom_gltf_navmesh':(BuildContext context) {
                return const Demo();
              },
              '/custom_navmesh_generation':(BuildContext context) {
                return const Demo();
              },
              '/doors_and_keys':(BuildContext context) {
                return const Demo();
              },
              '/dynamic_navmesh':(BuildContext context) {
                return const Demo();
              },
              '/dynamic_obstacles':(BuildContext context) {
                return const Demo();
              },
              '/find_diverse_path':(BuildContext context) {
                return const Demo();
              },
              '/find_nearest_poly':(BuildContext context) {
                return const Demo();
              },
              '/find_node_path':(BuildContext context) {
                return const Demo();
              },
              '/find_path':(BuildContext context) {
                return const Demo();
              },
              '/find_random_point':(BuildContext context) {
                return const Demo();
              },
              '/find_shortest_path':(BuildContext context) {
                return const Demo();
              },
              '/find_smooth_path':(BuildContext context) {
                return const Demo();
              },
              '/find_straight_path':(BuildContext context) {
                return const Demo();
              },
              '/flood_fill_pruning':(BuildContext context) {
                return const Demo();
              },
              '/flow_field_pathfinding':(BuildContext context) {
                return const Demo();
              },
              '/fps_dynamic_navmesh':(BuildContext context) {
                return const Demo();
              },
              '/mark_compact_heightfield_areas':(BuildContext context) {
                return const Demo();
              },
              '/move_along_surface':(BuildContext context) {
                return const Demo();
              },
              '/multiple_agent_sizes':(BuildContext context) {
                return const Demo();
              },
              '/navmesh_constrained_charcter_controller':(BuildContext context) {
                return const Demo();
              },
              '/off_mesh_connections':(BuildContext context) {
                return const Demo();
              },
              '/rasterize_filled_volume':(BuildContext context) {
                return const Demo();
              },
              '/raycast':(BuildContext context) {
                return const Demo();
              },
              '/solo_navmesh':(BuildContext context) {
                return const SoloNavmesh();
              },
              '/tiled_navmesh':(BuildContext context) {
                return const Demo();
              },
              '/upload_model':(BuildContext context) {
                return const Demo();
              },
            }
          ),
        )
      )
    );
  }
}

@immutable
class AppBar extends StatelessWidget{
  const AppBar({
    super.key,
    required this.page,
    required this.callback
  });
  final String page;
  final void Function(String page,[double? loc]) callback;
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return Container(
      height: 45,
      padding: const EdgeInsets.only(left: 10),
      color: Theme.of(context).cardColor,
      child: Row(
        children: [
          InkWell(
            onTap: (){
              callback('');
            },
            child: const Icon(
              Icons.arrow_back_ios_new_rounded
            ),
          ),
          const SizedBox(width: 20,),
          Text(
            (page[0]+page.substring(1)).replaceAll('_', ' ').toUpperCase(),
            style: Theme.of(context).primaryTextTheme.bodyMedium,
          )
        ],
      ),
    );
  }
}

class Examples extends StatefulWidget{
  const Examples({
    super.key,
    required this.callback,
    required this.prevLocation
  });

  final void Function(String page,[double? location]) callback;
  final double prevLocation;

  @override
  ExamplesPageState createState() => ExamplesPageState();
}

class ExamplesPageState extends State<Examples> {
  double deviceHeight = double.infinity;
  double deviceWidth = double.infinity;
  ScrollController controller = ScrollController();

  List<Widget> displayExamples(){
    List<Widget> widgets = [];

    double response = CSS.responsive(width: 480);

    for(int i = 0;i < filesJson.length;i++){
      widgets.add(
        InkWell(
          onTap: (){
            widget.callback(filesJson[i],controller.offset);
          },
          child: Container(
            margin: const EdgeInsets.all(10),
            width: response-65,
            height: response,
            decoration: BoxDecoration(
              color: Theme.of(context).cardColor,
              borderRadius: const BorderRadius.all(Radius.circular(10)),
              boxShadow: [
                BoxShadow(
                  color: Theme.of(context).shadowColor,
                  blurRadius: 5,
                  offset: const Offset(2, 2),
                ),
              ]
            ),
            child: Column(
              children:[
                Container(
                  width: response,
                  height: response-65,
                  decoration: BoxDecoration(
                    image: DecorationImage(
                      image: ExactAssetImage('assets/screenshots/${filesJson[i]}.png'),
                      fit: BoxFit.cover,
                    ),
                    borderRadius: const BorderRadius.only(topRight:Radius.circular(10),topLeft:Radius.circular(10)),
                  ),
                ),
                const SizedBox(height: 10),
                Text(
                  filesJson[i].replaceAll('_',' ').toUpperCase(),
                  style: Theme.of(context).primaryTextTheme.bodyMedium,
                )
              ]
            )
          )
        )
      );
    }

    return widgets;
  }

  @override
  void initState(){
    WidgetsBinding.instance.addPostFrameCallback((timeStamp) { 
      controller.jumpTo(widget.prevLocation);
    });
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    deviceHeight = MediaQuery.of(context).size.height;
    deviceWidth = MediaQuery.of(context).size.width;
    
    return SingleChildScrollView(
      controller: controller,
      child: Wrap(
        runAlignment: WrapAlignment.spaceBetween,
        alignment: WrapAlignment.center,
        crossAxisAlignment: WrapCrossAlignment.center,
        children: displayExamples(),
      )
    );
  }
}