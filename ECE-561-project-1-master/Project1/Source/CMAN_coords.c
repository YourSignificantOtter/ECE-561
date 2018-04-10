#include "Geometry.h"
								
const PT_T waypoints_old[] = {	//	Lat 		Lon		Name		
	{	56.07	,	152.57	,	"ALBATROSS BNK"	},	
	{	51.15	,	-179	,	"AMCHITKA"	},	
	{	59.38	,	153.35	,	"AUGUSTINE ISLAND, AK"	},	
	{	22.02	,	94.05	,	"BAY CAMPECHE"	},	
	{	57.07	,	177.75	,	"BERING SEA"	},	
	{	30.09	,	88.77	,	"BILOXI"	},	
	{	60.84	,	146.88	,	"BLIGH REEF LIGHT, AK"	},	
	{	38.24	,	123.3	,	"BODEGA BAY"	},	
	{	42.35	,	70.65	,	"BOSTON"	},	
	{	41.39	,	71	,	"BUZZARDS BAY"	},	
	{	41.4	,	71.03	,	"BUZZARDS BAY,MA"	},	
	{	15.09	,	75.06	,	"C CARIBBEAN"	},	
	{	59.76	,	152.09	,	"C COOK INL"	},	
	{	55.85	,	142.56	,	"C GULF ALASKA"	},	
	{	38.03	,	130	,	"CALIFORNIA"	},	
	{	28.91	,	78.47	,	"CANAVERAL E"	},	
	{	28.52	,	80.17	,	"CANAVERAL W"	},	
	{	43.34	,	124.38	,	"CAPE ARAGO ,OR"	},	
	{	59.5	,	147.98	,	"CAPE CLEARE"	},	
	{	47.35	,	124.73	,	"CAPE ELIZABETH"	},	
	{	34.62	,	76.53	,	"CAPE LOOKOUT, NC"	},	
	{	59.69	,	143.4	,	"CAPE SUCKLING"	},	
	{	29.14	,	83.03	,	"CEDAR KEY, FL"	},	
	{	36.91	,	75.71	,	"CHESAPEAKE LIGHT, VA"	},	
	{	46.14	,	124.51	,	"COL RIVER BAR"	},	
	{	30.25	,	88.07	,	"DAUPHIN ISLAND, AL"	},	
	{	38.46	,	74.7	,	"DELAWARE BAY"	},	
	{	47.68	,	124.49	,	"DESTRUCTION ISLAND,WA"	},	
	{	47.08	,	90.73	,	"DEVILS ISLAND, WI"	},	
	{	35.01	,	75.4	,	"DIAMOND SHLS"	},	
	{	60.55	,	152.14	,	"DRIFT RIVER TERMINAL,"	},	
	{	42.49	,	79.35	,	"DUNKIRK, NY"	},	
	{	23.87	,	70.87	,	"E BAHAMAS"	},	
	{	15.01	,	67.5	,	"E CARIBBEAN"	},	
	{	16.5	,	63.5	,	"E CARIBBEAN"	},	
	{	25.97	,	85.59	,	"E GULF MEXICO"	},	
	{	47.58	,	86.59	,	"E LK SUPERIOR"	},	
	{	34.25	,	119.84	,	"E STA BARBARA"	},	
	{	58.92	,	151.95	,	"EAST AMATULI ISLAND L"	},	
	{	32.5	,	79.1	,	"EDISTO"	},	
	{	40.75	,	124.58	,	"EEL RIVER"	},	
	{	26.97	,	96.7	,	"EILEEN"	},	
	{	58.25	,	137.99	,	"FAIRWEATHER"	},	
	{	57.27	,	133.63	,	"FIVE FINGERS, AK"	},	
	{	59.33	,	152.00	,	"FLAT ISLAND LIGHT"	},	
	{	32.69	,	79.89	,	"FOLLY ISLAND, SC"	},	
	{	25.59	,	80.10	,	"FOWEY ROCK, FL"	},	
	{	33.44	,	77.74	,	"FRYING PAN SHLS"	},	
	{	29.23	,	94.41	,	"GALVESTON"	},	
	{	41.11	,	66.58	,	"GEORGES BNK"	},	
	{	31.4	,	80.87	,	"GRAYS REEF"	},	
	{	56.3	,	148.2	,	"GULF ALASKA"	},	
	{	43.19	,	69.14	,	"GULF MAINE"	},	
	{	37.36	,	122.88	,	"HALF MOON BAY"	},	
	{	42.97	,	70.62	,	"ISLE OF SHOALS, NH"	},	
	{	44.27	,	67.31	,	"JONESPORT"	},	
	{	29.82	,	83.59	,	"KEATON BEACH, FL"	},	
	{	58.04	,	149.99	,	"KENNEDY ENTR"	},	
	{	52.74	,	154.96	,	"KODIAK"	},	
	{	42.47	,	82.76	,	"LAKE ST. CLAIR"	},	
	{	27.9	,	95.39	,	"LANEILLE"	},	
	{	43.62	,	77.41	,	"LK ONTARIO"	},	
	{	40.25	,	73.17	,	"LONG ISLAND"	},	
	{	24.84	,	80.86	,	"LONG KEY, FL"	},	
	{	14.36	,	46.01	,	"M ATLANTIC"	},	
	{	25.9	,	89.67	,	"M GULF MEXICO"	},	
	{	48.06	,	87.79	,	"M LK SUPERIOR"	},	
	{	43.78	,	68.86	,	"MATINICUS ROCK, ME"	},	
	{	61.08	,	146.66	,	"MIDDLE ROCK LIGHT, AK"	},	
	{	25.01	,	80.38	,	"MOLASSES REEF, FL"	},	
	{	59.93	,	147.99	,	"MONTAGUE STR"	},	
	{	40.69	,	72.05	,	"MONTAUK PT"	},	
	{	36.79	,	122.4	,	"MONTEREY BAY"	},	
	{	43.97	,	68.13	,	"MT DESERT ROCK, ME"	},	
	{	23.55	,	154.06	,	"N HAWAII"	},	
	{	23.56	,	153.9	,	"N HAWAII"	},	
	{	45.34	,	86.41	,	"N LK MICHIGAN"	},	
	{	40.5	,	69.25	,	"NANTUCKET"	},	
	{	41.44	,	70.19	,	"NANTUCKET SND"	},	
	{	27.47	,	71.49	,	"NE BAHAMAS"	},	
	{	48.49	,	124.73	,	"NEAH BAY"	},	
	{	48.33	,	123.17	,	"NEW DUNGENESS"	},	
	{	44.61	,	124.07	,	"NEWPORT, OR"	},	
	{	23.45	,	162.28	,	"NW HAWAII"	},	
	{	24.32	,	162.06	,	"NW HAWAII"	},	
	{	40.37	,	73.7	,	"NYC ENTR"	},	
	{	34.48	,	77.28	,	"ONSLOW BAY"	},	
	{	30.07	,	87.55	,	"ORANGE BCH"	},	
	{	48.22	,	88.37	,	"PASSAGE ISLAND, MI"	},	
	{	59.74	,	149.47	,	"PILOT ROCK, AK"	},	
	{	38.96	,	123.74	,	"POINT ARENA, CA"	},	
	{	34.58	,	120.65	,	"POINT ARGUELLO, CA"	},	
	{	24.69	,	82.77	,	"POLASKI SHOALS, FL"	},	
	{	27.83	,	97.05	,	"PORT ARANSAS, TX"	},	
	{	42.75	,	124.82	,	"PORT ORFORD"	},	
	{	43.53	,	70.14	,	"PORTLAND"	},	
	{	61.06	,	146.70	,	"POTATO POINT, AK"	},	
	{	39.2	,	123.97	,	"PT ARENA"	},	
	{	34.7	,	120.96	,	"PT ARGUELLO"	},	
	{	47.87	,	89.31	,	"ROCK OF AGES, MI"	},	
	{	51.63	,	172.17	,	"S ALEUTIANS"	},	
	{	20.99	,	65.01	,	"S ATLANTIC"	},	
	{	21.65	,	58.7	,	"S ATLANTIC"	},	
	{	27.5	,	63	,	"S ATLANTIC"	},	
	{	59.05	,	152.22	,	"S COOK INL"	},	
	{	32.38	,	75.41	,	"S HATTERAS"	},	
	{	44.28	,	82.42	,	"S LK HURON"	},	
	{	42.67	,	87.03	,	"S LK MICHIGAN"	},	
	{	28.79	,	86.01	,	"S PENSACOLA"	},	
	{	33.67	,	120.2	,	"S STA ROSA"	},	
	{	29.68	,	94.03	,	"SABINE, TX"	},	
	{	32.49	,	118.03	,	"SAN CLEMENTE"	},	
	{	37.76	,	122.83	,	"SAN FRANCISCO"	},	
	{	35.74	,	121.88	,	"SAN MARTIN"	},	
	{	24.4	,	81.93	,	"SAND KEY"	},	
	{	24.45	,	81.88	,	"SAND KEY, FL"	},	
	{	55.01	,	171.98	,	"SE BERING SEA"	},	
	{	41.26	,	69.31	,	"SE CAPE COD"	},	
	{	17.53	,	152.38	,	"SE HAWAII"	},	
	{	60.23	,	146.83	,	"SEAL ROCKS"	},	
	{	26.7	,	78.99	,	"SETTLEMENT PT, GBI"	},	
	{	43.75	,	87.69	,	"SHEBOYGAN, WI"	},	
	{	57.92	,	154.25	,	"SHELIKOF STR"	},	
	{	53.93	,	160.81	,	"SHUMAGIN IS"	},	
	{	56.63	,	136.15	,	"SITKA SND"	},	
	{	48.32	,	122.84	,	"SMITH ISLAND, WA"	},	
	{	24.63	,	81.11	,	"SOMBRERO KEY, FL"	},	
	{	41.63	,	82.84	,	"SOUTH BASS ISLAND, OH"	},	
	{	28.91	,	89.43	,	"SOUTHWEST PASS , LA"	},	
	{	30.04	,	80.53	,	"ST AUGUSTINE"	},	
	{	41.85	,	124.38	,	"ST GEORGES"	},	
	{	29.86	,	81.27	,	"ST. AUGUSTINE, FL"	},	
	{	29.41	,	84.86	,	"ST. GEORGE OFFSHORE"	},	
	{	34.27	,	120.46	,	"STA BARBARA"	},	
	{	34.87	,	120.86	,	"STA MARIA"	},	
	{	33.74	,	119.06	,	"STA MONICA"	},	
	{	47.18	,	87.23	,	"STANNARD ROCK, MI"	},	
	{	44.64	,	124.5	,	"STONEWALL BNK"	},	
	{	40.89	,	137.45	,	"SW ASTORIA"	},	
	{	55	,	-175.28	,	"SW BERING SEA"	},	
	{	17.09	,	157.81	,	"SW HAWAII"	},	
	{	32.43	,	119.53	,	"TANNER BANK"	},	
	{	48.39	,	124.74	,	"TATOOSH ISLAND, WA"	},	
	{	39.58	,	72.6	,	"TEXAS TWR 4"	},	
	{	38.9	,	76.44	,	"THOMAS POINT, MD"	},	
	{	45.89	,	125.83	,	"TILLAMOOK"	},	
	{	34.21	,	76.95	,	"UNCW"	},	
	{	45.35	,	82.84	,	"V N LK HURON"	},	
	{	27.07	,	82.45	,	"VENICE, FL"	},	
	{	36.61	,	74.84	,	"VIRGINIA BCH"	},	
	{	46.01	,	130.97,	"W ASTORIA"	},	
	{	14.48	,	53.01	,	"W ATLANTIC"	},	
	{	31.98	,	69.65	,	"W BERMUDA"	},	
	{	16.83	,	81.5	,	"W CARIBBEAN"	},	
	{	25.79	,	93.67	,	"W GULF MEXICO"	},	
	{	19.09	,	160.66	,	"W HAWAII"	},	
	{	41.68	,	82.4	,	"W LK ERIE"	},	
	{	47.33	,	89.79	,	"W LK SUPERIOR"	},	
	{	60.6	,	146.84	,	"W ORCA BAY"	},	
	{	42.57	,	130.46	,	"W OREGON"	},	
	{	60.79	,	148.28	,	"W PWS"	},	
	{	28.5	,	84.52	,	"W TAMPA"	},	
	{	47.66	,	122.44	,	"WEST POINT, WA"	},	
	{	19.87	,	85.06	,	"YUCATAN CHNL"	},	
	{	0	,	0	,	"END"	},	};
